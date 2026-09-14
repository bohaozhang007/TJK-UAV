"""Offline visual synchronization of v21 detection logs and FPV recordings.

Requires numpy, opencv-python and FFmpeg. Does not contact the robot.

Example (run from repository root):
  python scripts/replace_detection_frames.py --video recording.mkv --imgs LOG_DIR/vis

Output defaults to LOG_DIR/merge.mp4; --output overrides it. Use --analyze-only
for a synchronization report without encoding. Existing videos are not overwritten.
All image types, including track, require paired JSON with timestamp_s (image capture
Unix seconds, on the same clock as detection metadata). Images are resolved using
image_file or the JSON stem (track_00.json -> track_00.png, for example).
All timestamped images share one clock-and-visual alignment pipeline. Images without
metadata are reported and skipped; there is no visual-only fallback. Static/ambiguous
matches and unsupported clock discontinuities are conservatively rejected.
Output is H.264 MP4, resampled to source nominal FPS using decoded timestamps;
each accepted image occupies one frame. This is an offline visualization tool,
not a guarantee of exact sensor synchronization. Distorted/cropped images or
different camera views may require another matching method.
"""
import argparse
import json
from pathlib import Path
import subprocess

import cv2
import numpy as np

from record_fpv import find_ffmpeg


def read_image(path):
    im = cv2.imdecode(np.fromfile(str(path), dtype=np.uint8), cv2.IMREAD_COLOR)
    if im is None:
        raise ValueError(f"Cannot read image: {path}")
    return im


def feature(im):
    # Discard the status-label strip; blur suppresses thin boxes and text.
    gray = cv2.cvtColor(im, cv2.COLOR_BGR2GRAY)
    gray = gray[round(gray.shape[0] * .12):]
    small = cv2.resize(gray, (64, 36), interpolation=cv2.INTER_AREA)
    v = cv2.GaussianBlur(small, (3, 3), 0).astype(np.float32).ravel()
    v -= v.mean()
    return v / max(float(np.linalg.norm(v)), 1e-6)


def load_records(directory):
    records = []
    for path in sorted(directory.rglob('*.json')):
        meta = json.loads(path.read_text(encoding='utf-8'))
        if 'timestamp_s' not in meta:
            continue
        names = [meta.get('image_file', ''), path.stem + '_top3.png', path.stem + '.jpg', path.stem + '.png',
                 path.stem.removesuffix('_trigger') + '_top3_trigger.png']
        image = next((path.parent / n for n in names if n and (path.parent / n).is_file()), None)
        if image:
            records.append(dict(image=str(image.resolve()), timestamp_s=float(meta['timestamp_s']),
                                kind=image_kind(image), alignment_method='clock_and_visual'))
    records.sort(key=lambda r: r['timestamp_s'])
    if len(records) < 4:
        raise ValueError('Need at least four timestamped detection images with JSON metadata')
    return records


def image_kind(path):
    name = Path(path).name
    if name.startswith('track_'):
        return 'track'
    if name.startswith('reacquire_'):
        return 'reacquire'
    if name.startswith('trigger.') or '_trigger.' in name:
        return 'trigger'
    return 'patrol' if name.startswith('patrol_') else 'snapshot'


def unpaired_images(directory, records):
    known = {Path(r['image']).resolve() for r in records}
    return [dict(image=str(p.resolve()), kind=image_kind(p), status='skipped',
                 reason='missing paired JSON with timestamp_s')
            for p in sorted(directory.rglob('*'))
            if p.is_file() and p.suffix.lower() in {'.png', '.jpg', '.jpeg'}
            and p.resolve() not in known]


def replacement_slots(rows, fps):
    replacements = {}
    priority = {'snapshot': 0, 'patrol': 1, 'trigger': 2, 'reacquire': 3, 'track': 4}
    for row in rows:
        if row['status'] != 'matched':
            continue
        slot = int(round(row['video_s'] * fps))
        row['output_frame'] = slot
        old = replacements.get(slot)
        rank = lambda r: (priority.get(r.get('kind'), 0), r['score'])
        if old is None or rank(row) > rank(old):
            if old is not None:
                old.update(status='superseded', reason='another annotation of the same output frame')
            replacements[slot] = row
        else:
            row.update(status='superseded', reason='another annotation of the same output frame')
    return replacements


def index_video(path):
    cap = cv2.VideoCapture(str(path))
    if not cap.isOpened():
        raise ValueError(f'Cannot open {path}')
    fps = cap.get(cv2.CAP_PROP_FPS)
    vectors, times = [], []
    while True:
        ok, frame = cap.read()
        if not ok:
            break
        vectors.append(feature(frame))
        times.append(cap.get(cv2.CAP_PROP_POS_MSEC) / 1000.)
    cap.release()
    times = np.asarray(times)
    if len(times) < 4 or not np.isfinite(times).all() or np.any(np.diff(times) <= 0):
        raise ValueError(f'Missing/nonmonotonic video timestamps: {path}')
    times -= times[0]
    print(f'Indexed {path.name}: {len(times)} frames, {times[-1]:.2f}s', flush=True)
    return np.asarray(vectors), times, fps


def fit_clock(x, y, tolerance=.35):
    """Deterministic RANSAC; permit at most 2% clock-rate difference."""
    best = None
    for i in range(len(x)):
        for j in range(i + 1, len(x)):
            if x[j] - x[i] < 10:
                continue
            a = (y[j] - y[i]) / (x[j] - x[i])
            if not .98 <= a <= 1.02:
                continue
            b = y[i] - a * x[i]
            mask = np.abs(y - (a * x + b)) <= tolerance
            if best is None or mask.sum() > best.sum():
                best = mask
    if best is None or best.sum() < 4:
        raise ValueError('Insufficient consistent visual anchors to synchronize')
    a, b = np.polyfit(x[best], y[best], 1)
    mask = np.abs(y - (a * x + b)) <= tolerance
    if mask.sum() < 4 or np.ptp(x[mask]) < 10 or not .98 <= a <= 1.02:
        raise ValueError('Unstable synchronization fit')
    a, b = np.polyfit(x[mask], y[mask], 1)
    return float(a), float(b), mask


def synchronize(records, queries, matrix, times, min_score, min_margin):
    scores = queries @ matrix.T
    peaks = scores.argmax(axis=1)
    strong = []
    for i, k in enumerate(peaks):
        outside = np.abs(times - times[k]) > 2.
        margin = float(scores[i, k] - scores[i, outside].max()) if outside.any() else 0.
        if scores[i, k] >= min_score and margin >= min_margin:
            strong.append(i)
    if len(strong) < 4:
        raise ValueError(f'Only {len(strong)} unambiguous visual anchors')
    origin = records[0]['timestamp_s']
    x = np.array([r['timestamp_s'] - origin for r in records])
    strong = np.asarray(strong)
    a, b, mask = fit_clock(x[strong], times[peaks[strong]])
    anchors = strong[mask]
    if len(anchors) < max(4, len(strong) * .6):
        raise ValueError('Conflicting anchors: possible timestamp discontinuity; manual review needed')
    lo, hi = float(x[anchors].min()), float(x[anchors].max())
    residuals = times[peaks[anchors]] - (a * x[anchors] + b)
    output = []
    for i, record in enumerate(records):
        pred = float(a * x[i] + b)
        row = dict(record, predicted_video_s=pred, status='skipped')
        window = np.flatnonzero(np.abs(times - pred) <= .4)
        if x[i] < lo - 3 or x[i] > hi + 3:
            row['reason'] = 'outside validated anchor span'
        elif not len(window):
            row['reason'] = 'outside video'
        else:
            k = int(window[np.argmax(scores[i, window])])
            score = float(scores[i, k])
            # Flat local maxima are temporally ambiguous even with a good clock fit.
            plausible = window[scores[i, window] >= score - .001]
            ambiguity = float(np.ptp(times[plausible]))
            row.update(video_frame=k, video_s=float(times[k]), score=score,
                       local_ambiguity_s=ambiguity)
            if score < min_score:
                row['reason'] = 'low visual similarity'
            elif abs(times[k] - pred) > .3:
                row['reason'] = 'visual match disagrees with clock fit'
            elif ambiguity > .2:
                row['reason'] = 'repetitive or static scene'
            else:
                row['status'] = 'matched'
        output.append(row)
    return dict(timestamp_origin_s=origin, clock_scale=a, video_offset_s=b,
                formula='video_s = clock_scale * (timestamp_s - timestamp_origin_s) + video_offset_s',
                anchor_count=len(anchors), anchor_span_s=[lo, hi],
                anchor_residual_p95_s=float(np.percentile(np.abs(residuals), 95)), frames=output)


def render(video, output, report, times, fps, ffmpeg):
    if not ffmpeg:
        raise ValueError('FFmpeg not found; pass --ffmpeg PATH')
    if output.exists():
        raise ValueError(f'Refusing to overwrite {output}')
    cap = cv2.VideoCapture(str(video))
    ok, current = cap.read()
    if not ok:
        raise ValueError('Cannot decode video for rendering')
    height, width = current.shape[:2]
    duration = float(times[-1] + np.median(np.diff(times)))
    count = int(np.ceil(duration * fps))
    replacements = replacement_slots(report['frames'], fps)
    output.parent.mkdir(parents=True, exist_ok=True)
    partial = output.with_name(output.stem + '.partial.mp4')
    if partial.exists():
        raise ValueError(f'Refusing to overwrite {partial}')
    command = [ffmpeg, '-hide_banner', '-loglevel', 'error', '-n', '-f', 'rawvideo',
               '-pix_fmt', 'bgr24', '-s', f'{width}x{height}', '-r', str(fps), '-i', '-',
               '-i', str(video), '-map', '0:v:0', '-map', '1:a?', '-c:v', 'libx264',
               '-preset', 'fast', '-crf', '18', '-pix_fmt', 'yuv420p', '-profile:v', 'high',
               '-tag:v', 'avc1', '-c:a', 'aac', '-t', str(count / fps),
               '-movflags', '+faststart', str(partial)]
    log = output.with_suffix('.ffmpeg.log')
    proc = None
    try:
        with log.open('wb') as err:
            proc = subprocess.Popen(command, stdin=subprocess.PIPE, stderr=err)
            source_index = 0
            ok_next, upcoming = cap.read()
            for n in range(count):
                t = n / fps
                while ok_next and source_index + 1 < len(times) and times[source_index + 1] <= t:
                    current = upcoming
                    source_index += 1
                    ok_next, upcoming = cap.read()
                if not ok_next and source_index < len(times) - 1:
                    raise ValueError('Unexpected early video decode failure')
                frame = current
                if n in replacements:
                    frame = read_image(replacements[n]['image'])
                    if abs(frame.shape[1] / frame.shape[0] - width / height) > .03:
                        raise ValueError('Detection image aspect ratio differs from video')
                    frame = cv2.resize(frame, (width, height))
                proc.stdin.write(frame.tobytes())
                if n % 1800 == 0:
                    print(f'Rendering {n}/{count}', flush=True)
            proc.stdin.close()
            if proc.wait() != 0:
                raise RuntimeError(f'FFmpeg failed; see {log}')
        partial.rename(output)
    finally:
        cap.release()
        if proc is not None and proc.poll() is None:
            proc.kill()
            proc.wait()
    report.update(output=str(output), output_fps=fps, output_frames=count,
                  replaced_frames=len(replacements), output_duration_s=count / fps,
                  source_duration_s=duration,
                  timing_note='CFR resampling from source PTS; duration rounded up by less than one output frame. Each detection replaces one output frame. Audio retained if present.')


def parse_args(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--video', type=Path, required=True, help='Input video file (.mp4 or .mkv)')
    parser.add_argument('--imgs', type=Path, required=True, help='vis folder; subfolders are read recursively')
    parser.add_argument('--output', type=Path, help='Output MP4; default: merge.mp4 beside the vis folder')
    parser.add_argument('--analyze-only', action='store_true')
    parser.add_argument('--ffmpeg', default=find_ffmpeg())
    parser.add_argument('--min-score', type=float, default=.94)
    parser.add_argument('--min-margin', type=float, default=.01)
    args = parser.parse_args(argv)
    args.video = args.video.resolve()
    args.imgs = args.imgs.resolve()
    args.output = args.output.resolve() if args.output else args.imgs.parent / 'merge.mp4'
    if not args.video.is_file():
        parser.error('--video must point to an existing video file')
    if not args.imgs.is_dir():
        parser.error('--imgs must point to an existing vis folder')
    if args.output.suffix.lower() != '.mp4':
        parser.error('--output must end in .mp4')
    if args.output == args.video:
        parser.error('Output cannot be the input video')
    if args.output.exists() and not args.analyze_only:
        parser.error('Output already exists; choose a new filename with --output')
    return args


def main(argv=None):
    args = parse_args(argv)
    cv2.setNumThreads(2)
    records = load_records(args.imgs)
    skipped = unpaired_images(args.imgs, records)
    print(f'Loaded {len(records)} timestamped images; skipped {len(skipped)} without metadata', flush=True)
    queries = np.asarray([feature(read_image(r['image'])) for r in records])
    args.output.parent.mkdir(parents=True, exist_ok=True)
    report_path = args.output.with_suffix('.sync.json')
    try:
        matrix, times, fps = index_video(args.video)
        report = synchronize(records, queries, matrix, times, args.min_score, args.min_margin)
        matched = sum(r['status'] == 'matched' for r in report['frames'])
        if not matched:
            raise ValueError('No sufficiently certain matches')
    except ValueError as exc:
        report_path.write_text(json.dumps(dict(video=str(args.video), error=str(exc),
                                              skipped_images=skipped), indent=2), encoding='utf-8')
        raise SystemExit(f'Synchronization failed: {exc}. See {report_path}') from exc
    print(f'{args.video.name}: {matched}/{len(records)} accepted, {report["anchor_count"]} anchors', flush=True)
    report.update(video=str(args.video), imgs=str(args.imgs), skipped_images=skipped,
                  input_images=len(records) + len(skipped), timestamped_images=len(records),
                  min_score=args.min_score, min_margin=args.min_margin)
    report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    if not args.analyze_only:
        render(args.video, args.output, report, times, fps, args.ffmpeg)
        report_path.write_text(json.dumps(report, ensure_ascii=False, indent=2), encoding='utf-8')
    print(f'Report: {report_path}', flush=True)


if __name__ == '__main__':
    main()
