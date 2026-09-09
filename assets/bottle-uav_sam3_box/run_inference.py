import json
import sys
import time
from pathlib import Path

import numpy as np
import torch
from PIL import Image, ImageDraw, ImageFont

ROOT = Path('C:/Users/colab999/Desktop/project')
sys.path.insert(0, str(ROOT / 'sam3'))
from sam3.model_builder import build_sam3_image_model
from sam3.model.sam3_image_processor import Sam3Processor

out_dir = Path(__file__).resolve().parent
image_path = ROOT / 'TJK-UAV/assets/bottle-uav.jpg'
im = Image.open(image_path).convert('RGB')
print('Loading local checkpoint...', flush=True)
start = time.perf_counter()
model = build_sam3_image_model(checkpoint_path=str(ROOT / 'sam3/sam3.pt'), load_from_HF=False, device='cuda')
processor = Sam3Processor(model, confidence_threshold=0.5)
print('Running text prompt: box', flush=True)
torch.cuda.synchronize()
inference_start = time.perf_counter()
with torch.inference_mode(), torch.autocast('cuda', dtype=torch.bfloat16):
    state = processor.set_image(im)
    result = processor.set_text_prompt(prompt='box', state=state)
torch.cuda.synchronize()
elapsed = time.perf_counter() - inference_start
boxes = result['boxes'].float().cpu().numpy()
scores = result['scores'].float().cpu().numpy()
masks = result['masks'].cpu().numpy().reshape(-1, im.height, im.width)
order = np.argsort(-scores)
canvas = np.array(im).copy()
colors = [(255, 55, 55), (40, 210, 70), (30, 140, 255), (255, 180, 20)]
detections = []
for i, idx in enumerate(order):
    color = colors[i % len(colors)]
    mask = masks[idx]
    canvas[mask] = (0.65 * canvas[mask] + 0.35 * np.array(color)).astype(np.uint8)
    Image.fromarray(mask.astype(np.uint8) * 255).save(out_dir / f'mask_{i+1}.png')
    detections.append({'id': i+1, 'score': float(scores[idx]), 'bbox_xyxy': boxes[idx].tolist(), 'mask': f'mask_{i+1}.png'})
vis = Image.fromarray(canvas)
draw = ImageDraw.Draw(vis)
font = ImageFont.truetype('C:/Windows/Fonts/arial.ttf', 24)
for i, det in enumerate(detections):
    box = det['bbox_xyxy']
    color = colors[i % len(colors)]
    draw.rectangle(box, outline=color, width=4)
    pos = (max(0, box[0]), max(0, box[1]-32))
    label = f"box #{det['id']} {det['score']:.3f}"
    bounds = draw.textbbox(pos, label, font=font)
    draw.rectangle((bounds[0]-2, bounds[1]-3, bounds[2]+3, bounds[3]+3), fill=color)
    draw.text(pos, label, font=font, fill='white')
vis.save(out_dir / 'visualization.jpg', quality=95)
report = {'image': str(image_path), 'prompt': 'box', 'confidence_threshold': 0.5, 'precision': 'bfloat16_amp', 'image_size': list(im.size), 'inference_seconds': elapsed, 'detections': detections}
(out_dir / 'results.json').write_text(json.dumps(report, indent=2), encoding='utf-8')
(out_dir / 'boxes.txt').write_text(''.join(' '.join(str(round(v)) for v in d['bbox_xyxy']) + '\n' for d in detections), encoding='ascii')
print(json.dumps(report, indent=2), flush=True)
print(f'Total elapsed: {time.perf_counter()-start:.2f}s', flush=True)
