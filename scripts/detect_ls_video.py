"""Offline SAM3 visual-reference detection; run with the sam3 environment."""
from __future__ import annotations

import argparse
import json
import math
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np
from PIL import Image


def write_json(path, value):
    temp = path.with_suffix('.tmp')
    temp.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding='utf-8')
    temp.replace(path)


def save_image(path, image):
    if not cv2.imwrite(str(path), image, [cv2.IMWRITE_JPEG_QUALITY, 95]):
        raise OSError(f'Cannot write {path}')


def extract(data, output, fps):
    manifest_path = output / 'frames.json'
    if manifest_path.exists():
        manifest = json.loads(manifest_path.read_text(encoding='utf-8'))
        assert manifest['output_fps'] == fps
        assert all((output / x['file']).exists() for x in manifest['frames'])
        return manifest
    cap = cv2.VideoCapture(str(data / 'video.mp4'))
    source_fps = cap.get(cv2.CAP_PROP_FPS)
    count = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
    if not cap.isOpened() or source_fps <= 0 or count <= 0:
        raise RuntimeError('Cannot read video metadata')
    # Uniform CFR sampling: the source frame covering each output timestamp.
    total = math.ceil(count / source_fps * fps - 1e-8)
    frames = []
    source_index, frame = -1, None
    for index in range(total):
        wanted = min(int(math.floor(index * source_fps / fps + 1e-8)), count - 1)
        while source_index < wanted:
            ok, frame = cap.read()
            if not ok:
                raise RuntimeError(f'Video decode failed at source frame {source_index + 1}')
            source_index += 1
        name = f'frame_{index:06d}.jpg'
        save_image(output / name, frame)
        frames.append(dict(file=name, timestamp_s=index / fps, source_index=wanted))
    cap.release()
    manifest = dict(source_fps=source_fps, source_frames=count, output_fps=fps,
                    frame_count=total, sampling='floor(timestamp * source_fps)', frames=frames)
    write_json(manifest_path, manifest)
    return manifest


def frame_detections(boxes, scores, width, height, threshold, nms_iou):
    candidates = []
    for box, score in zip(boxes, scores):
        box = np.asarray(box, dtype=float)
        if not np.isfinite(box).all() or not np.isfinite(score) or score < threshold:
            continue
        # Exclude reference/padding detections, including boxes mostly outside the frame.
        cx, cy = (box[:2] + box[2:]) / 2
        clipped = np.clip(box, [0, 0, 0, 0], [width, height, width, height])
        area = np.prod(np.maximum(box[2:] - box[:2], 0))
        inside = np.prod(np.maximum(clipped[2:] - clipped[:2], 0))
        if not (0 <= cx < width and 0 <= cy < height) or inside <= 0 or inside < .8 * area:
            continue
        candidates.append((clipped, float(score)))
    candidates.sort(key=lambda x: x[1], reverse=True)
    kept = []
    for box, score in candidates:
        duplicate = False
        for previous, _ in kept:
            intersection = np.prod(np.maximum(np.minimum(box[2:], previous[2:]) -
                                              np.maximum(box[:2], previous[:2]), 0))
            union = np.prod(box[2:] - box[:2]) + np.prod(previous[2:] - previous[:2]) - intersection
            if intersection / max(union, 1e-9) > nms_iou:
                duplicate = True
                break
        if not duplicate:
            kept.append((box, score))
        if len(kept) == 3:
            break
    return [dict(rank=i + 1, box_xyxy=b.tolist(), confidence=s) for i, (b, s) in enumerate(kept)]


def label(image, text, xy, color=(0, 255, 255), scale=.8):
    cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 5, cv2.LINE_AA)
    cv2.putText(image, text, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main():
    parser = argparse.ArgumentParser(__doc__)
    parser.add_argument('--data', type=Path, default=Path(__file__).resolve().parents[1] / 'assets/ls')
    parser.add_argument('--sam3-root', type=Path, default=Path(__file__).resolve().parents[2] / 'sam3')
    parser.add_argument('--fps', type=float, default=1)
    parser.add_argument('--confidence', type=float, default=.3)
    parser.add_argument('--nms-iou', type=float, default=.5)
    parser.add_argument('--limit', type=int, default=0, help='Optional smoke-run frame count')
    args = parser.parse_args()
    if not math.isfinite(args.fps) or args.fps <= 0:
        parser.error('--fps must be finite and positive')
    data = args.data.resolve()
    fps_tag = f'{args.fps:g}fps'
    frames_dir, results_dir = data / f'frames_{fps_tag}', data / f'sam3_results_{fps_tag}'
    selected_dir = data / f'overviews_{fps_tag}_gt05'
    frames_dir.mkdir(exist_ok=True)
    results_dir.mkdir(exist_ok=True)
    selected_dir.mkdir(exist_ok=True)
    print('Extracting frames', flush=True)
    manifest = extract(data, frames_dir, args.fps)
    print(f"Extracted {manifest['frame_count']} frames", flush=True)
    sys.path.insert(0, str(args.sam3_root.resolve()))
    import torch
    from sam3.model_builder import build_sam3_image_model
    from sam3.model.sam3_image_processor import Sam3Processor
    from sam3.model.box_ops import box_cxcywh_to_xyxy

    class BoxProcessor(Sam3Processor):
        # Same official SAM3 box/score computation without unused mask upsampling.
        @torch.inference_mode()
        def _forward_grounding(self, state):
            outputs = self.model.forward_grounding(
                backbone_out=state['backbone_out'], find_input=self.find_stage,
                geometric_prompt=state['geometric_prompt'], find_target=None)
            scores = (outputs['pred_logits'].sigmoid() *
                      outputs['presence_logit_dec'].sigmoid().unsqueeze(1)).squeeze(-1)
            keep = scores > self.confidence_threshold
            boxes = box_cxcywh_to_xyxy(outputs['pred_boxes'][keep])
            scale = torch.tensor([state['original_width'], state['original_height']] * 2,
                                 device=self.device)
            state['boxes'], state['scores'] = boxes * scale, scores[keep]
            return state

    torch.set_num_threads(4)
    print('Loading SAM3 checkpoint', flush=True)
    model = build_sam3_image_model(device='cpu', checkpoint_path=str(args.sam3_root / 'sam3.pt'),
                                  load_from_HF=False, enable_segmentation=False,
                                  enable_inst_interactivity=False, compile=False).to('cuda').eval()
    processor = BoxProcessor(model, device='cuda', confidence_threshold=args.confidence)
    references = []
    sample = cv2.imread(str(frames_dir / manifest['frames'][0]['file']))
    h, w = sample.shape[:2]
    for name in ['t1', 't2', 't3', 't4', 't5']:
        image = cv2.imread(str(data / f'{name}.jpg'))
        box = np.array([float(v) for v in (data / f'{name}.txt').read_text().split()])
        rh, rw = image.shape[:2]
        assert box.shape == (4,) and 0 <= box[0] < box[2] <= rw and 0 <= box[1] < box[3] <= rh
        new_h = round(rh * w / rw)
        image = cv2.resize(image, (w, new_h), interpolation=cv2.INTER_AREA)
        mapped = box * [w / rw, new_h / rh, w / rw, new_h / rh] + [0, h, 0, h]
        x1, y1, x2, y2 = mapped
        prompt = [(x1 + x2) / (2 * w), (y1 + y2) / (2 * (h + new_h)),
                  (x2 - x1) / w, (y2 - y1) / (h + new_h)]
        references.append((name, image, mapped, prompt))
    settings = dict(fps=args.fps, confidence_threshold=args.confidence, nms_iou=args.nms_iou,
                    top_k=3, red_box_threshold_exclusive=0.5,
                    layout='frame above reference; reference scaled to frame width',
                    frame_size=[w, h], annotations_in_model_input=False,
                    checkpoint=str(args.sam3_root / 'sam3.pt'), total_frames=manifest['frame_count'])
    config_path = results_dir / 'settings.json'
    if config_path.exists():
        assert json.loads(config_path.read_text()) == settings, 'Settings changed; choose a new output directory'
    write_json(config_path, settings)
    started = time.time()
    total = min(args.limit or manifest['frame_count'], manifest['frame_count'])
    with torch.inference_mode(), torch.autocast('cuda', dtype=torch.bfloat16):
        visual_text = model.backbone.forward_text(['visual'], device='cuda')
        for index, entry in enumerate(manifest['frames'][:total]):
            folder = results_dir / Path(entry['file']).stem
            folder.mkdir(exist_ok=True)
            if (folder / 'detections.json').exists() and len(list(folder.glob('*.jpg'))) == 11:
                continue
            frame = cv2.imread(str(frames_dir / entry['file']))
            all_detections, panels = {}, []
            for name, reference, mapped, prompt in references:
                canvas = np.concatenate([frame, reference], axis=0)
                state = processor.set_image(Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
                state['backbone_out'].update(visual_text)
                state = processor.add_geometric_prompt(prompt, True, state)
                detections = frame_detections(state['boxes'].float().cpu().numpy(),
                    state['scores'].float().cpu().numpy(), w, h, args.confidence, args.nms_iou)
                all_detections[name] = dict(detections=detections, reference_box_composite_xyxy=mapped.tolist(),
                                           prompt_cxcywh_normalized=prompt)
                del state
                p = np.rint(mapped).astype(int)
                cv2.rectangle(canvas, tuple(p[:2]), tuple(p[2:]), (0, 255, 0), 3)
                label(canvas, f'{name} reference', (p[0], max(h + 25, p[1] - 8)))
                save_image(folder / f'{name}_input.jpg', canvas)
                annotated = frame.copy()
                for item in detections:
                    b = np.rint(item['box_xyxy']).astype(int)
                    color = (0, 0, 255) if item['confidence'] > 0.5 else (0, 255, 0)
                    cv2.rectangle(annotated, tuple(b[:2]), tuple(b[2:]), color, 2)
                    label(annotated, f"#{item['rank']} {item['confidence']:.3f}",
                          (min(b[0], w - 170), max(25, b[1] - 8)), scale=.65)
                if not detections:
                    label(annotated, 'No detection', (15, h - 20))
                save_image(folder / f'{name}_top3.jpg', annotated)
                label(annotated, name, (15, 35), scale=1)
                panels.append(annotated)
            overview = np.zeros((2 * h, 3 * w, 3), dtype=np.uint8)
            for k, panel in enumerate(panels):
                y, x = (k // 3) * h, (k % 3) * w
                overview[y:y+h, x:x+w] = panel
            label(overview, f'Frame {index:06d} | {entry["timestamp_s"]:.3f}s', (2*w + 15, h + 40))
            save_image(folder / 'overview.jpg', overview)
            write_json(folder / 'detections.json', dict(frame_index=index, **entry, references=all_detections))
            elapsed = time.time() - started
            progress = dict(completed=index+1, total=manifest['frame_count'], elapsed_s=elapsed,
                            last_frame=folder.name, status='running')
            write_json(results_dir / 'progress.json', progress)
            if index < 3 or (index + 1) % 10 == 0:
                print(f'{index+1}/{total} frames; {elapsed:.1f}s elapsed', flush=True)
    completed = sum((results_dir / Path(e['file']).stem / 'detections.json').exists() for e in manifest['frames'])
    # Rebuild selection from saved top3 metadata, including resumed frames.
    selected = []
    for entry in manifest['frames']:
        folder = results_dir / Path(entry['file']).stem
        if not (folder / 'detections.json').exists():
            continue
        record = json.loads((folder / 'detections.json').read_text(encoding='utf-8'))
        high = [dict(reference=name, **item) for name, ref in record['references'].items()
                for item in ref['detections'] if item['confidence'] > 0.5]
        if high:
            filename = f'{folder.name}_overview.jpg'
            shutil.copy2(folder / 'overview.jpg', selected_dir / filename)
            selected.append(dict(file=filename, timestamp_s=entry['timestamp_s'], detections=high))
    write_json(selected_dir / 'selection.json', dict(threshold_exclusive=0.5, count=len(selected), frames=selected))
    write_json(results_dir / 'progress.json', dict(completed=completed, total=manifest['frame_count'],
               selected_overviews=len(selected), elapsed_s=time.time()-started,
               status='complete' if completed == manifest['frame_count'] else 'partial'))
    print(f'Done: {completed}/{manifest["frame_count"]} frames', flush=True)


if __name__ == '__main__':
    main()
