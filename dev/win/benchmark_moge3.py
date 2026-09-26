"""Compare MoGe-3 FP32/FP16 after warming every input, without model or image I/O."""
import json
import os
import sys
import time
from datetime import datetime
from pathlib import Path

# Adaptive tuning can begin after 100 calls. Freeze its policy for both modes so
# measured samples never include autotuning; this does not change the server.
os.environ['FLEX_GEMM_AUTOTUNE_MODE'] = 'never'
os.environ['FLEX_GEMM_USE_AUTOTUNE_CACHE'] = '0'
os.environ['FLEX_GEMM_AUTOSAVE_AUTOTUNE_CACHE'] = '0'

import cv2
import numpy as np
import torch

MODEL_ROOT = Path(__file__).resolve().parents[3] / 'MoGe'
sys.path.insert(0, str(MODEL_ROOT))
from moge.model.v3 import MoGeModel


def main():
    paths = sorted(p for p in (MODEL_ROOT / 'example_images').iterdir()
                   if p.suffix.lower() in ('.jpg', '.png'))
    assert len(paths) == 10, 'Expected ten official example images'
    images = []
    for path in paths:
        rgb = cv2.cvtColor(cv2.imread(str(path)), cv2.COLOR_BGR2RGB)
        rgb = cv2.resize(rgb, (1920, 1080), interpolation=cv2.INTER_AREA)
        images.append(torch.from_numpy(rgb).permute(2, 0, 1).cuda().float() / 255.0)
    model = MoGeModel.from_pretrained(str(MODEL_ROOT / 'ckpts/moge-3-vit-l/model.pt')).cuda().eval()
    print(f'GPU: {torch.cuda.get_device_name(0)}; torch: {torch.__version__}', flush=True)

    def infer(image, fp16):
        # FP32 is exactly the official default invocation.
        return model.infer(image, use_fp16=True) if fp16 else model.infer(image)

    warmup = []
    for round_index in range(2):
        for fp16 in (False, True):
            start = time.perf_counter()
            for image in images:
                prediction = infer(image, fp16)
                torch.cuda.synchronize()
                del prediction
            seconds = time.perf_counter() - start
            warmup.append(dict(round=round_index + 1, use_fp16=fp16, images=10, seconds=seconds))
            print(f'Warmup {round_index+1}/2 fp16={fp16}: 10 images, {seconds:.3f}s (excluded)', flush=True)

    samples = []
    peak = {False: 0, True: 0}
    for repeat in range(10):
        for index, image in enumerate(images):
            # Reverse the first mode on alternate repetitions to balance ordering.
            modes = (False, True) if (repeat + index) % 2 == 0 else (True, False)
            for fp16 in modes:
                torch.cuda.synchronize()
                torch.cuda.reset_peak_memory_stats()
                start = time.perf_counter()
                prediction = infer(image, fp16)
                torch.cuda.synchronize()
                elapsed = time.perf_counter() - start
                peak[fp16] = max(peak[fp16], torch.cuda.max_memory_allocated())
                valid = prediction['mask'] & torch.isfinite(prediction['depth']) & (prediction['depth'] > 0)
                if not valid.any().item():
                    raise RuntimeError(f'No valid depth for {paths[index].name}, fp16={fp16}')
                samples.append(dict(repeat=repeat + 1, image=paths[index].name, use_fp16=fp16, seconds=elapsed))
                del prediction, valid
        print(f'Measured {10*(repeat+1)}/100 images per mode', flush=True)

    summary = {}
    for fp16 in (False, True):
        values = np.array([s['seconds'] for s in samples if s['use_fp16'] == fp16])
        summary[str(fp16)] = dict(count=len(values), mean_ms=values.mean()*1000,
                                 median_ms=np.median(values)*1000, p95_ms=np.percentile(values,95)*1000,
                                 std_ms=values.std()*1000, fps=1/values.mean(),
                                 first_50_mean_ms=values[:50].mean()*1000,
                                 last_50_mean_ms=values[50:].mean()*1000,
                                 peak_allocated_gib=peak[fp16]/2**30)
    report = dict(gpu=torch.cuda.get_device_name(0), torch=torch.__version__, cuda=torch.version.cuda,
                  width=1920, height=1080, distinct_images=10, repeats=10,
                  resolution_level=9, refine_steps=3, warmup_per_mode=20,
                  timing='synchronized wall time around infer; excludes loading, preprocessing, transfer, validation',
                  autotune='never, cache disabled for both modes; server configuration unchanged',
                  warmup=warmup, summary=summary,
                  fp16_speedup=summary['False']['mean_ms']/summary['True']['mean_ms'], samples=samples)
    target = MODEL_ROOT / 'infer_output' / f'fp16_benchmark_{datetime.now():%Y%m%d_%H%M%S}.json'
    target.parent.mkdir(parents=True, exist_ok=True)
    target.write_text(json.dumps(report, indent=2), encoding='utf-8')
    print(json.dumps(summary, indent=2), flush=True)
    print(f'FP16 speedup: {report["fp16_speedup"]:.3f}x\nReport: {target}', flush=True)


if __name__ == '__main__':
    main()
