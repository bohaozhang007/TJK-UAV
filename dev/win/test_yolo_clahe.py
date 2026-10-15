"""Reprocess saved images with CLAHE and the existing YOLO model process."""

import argparse
from contextlib import closing
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

from server import CLAHE_CLIP_LIMIT, CLAHE_TILE_GRID_SIZE, ModelProcess


DEFAULT_INPUT = Path(__file__).resolve().parents[2] / "logs/20261015_104041_425307/detector"
IMAGE_SUFFIXES = {".jpg", ".jpeg", ".png"}
BOX_COLOR = (255, 255, 0)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--input", type=Path, default=DEFAULT_INPUT)
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()
    images = sorted(
        path for path in args.input.iterdir()
        if (
            path.is_file()
            and path.suffix.lower() in IMAGE_SUFFIXES
        )
    )
    if not images:
        parser.error("No input images found")
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output_dir = args.output or args.input.parent / f"detector_clahe_yolo_{timestamp}"
    clahe = cv2.createCLAHE(
        clipLimit=CLAHE_CLIP_LIMIT,
        tileGridSize=CLAHE_TILE_GRID_SIZE,
    )
    saved = detected = failed = 0
    with closing(ModelProcess("yolo")) as detector:
        output_dir.mkdir(parents=True, exist_ok=False)
        print(f"Output: {output_dir}", flush=True)
        for index, path in enumerate(images, start=1):
            try:
                img = cv2.imdecode(np.fromfile(path, dtype=np.uint8), cv2.IMREAD_COLOR)
                if img is None:
                    raise ValueError("Failed to decode input image")
                lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
                lab[:, :, 0] = clahe.apply(lab[:, :, 0])
                img = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)
                detections = detector.request((img,))
                for item in detections:
                    x1, y1, x2, y2 = [round(value) for value in item["box"]]
                    cv2.rectangle(
                        img,
                        (x1, y1),
                        (x2, y2),
                        BOX_COLOR,
                        2,
                    )
                    cv2.putText(
                        img,
                        f"CLAHE {item['confidence']:.3f}",
                        (x1, max(20, y1 - 8)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.7,
                        BOX_COLOR,
                        2,
                        cv2.LINE_AA,
                    )
                ok, encoded = cv2.imencode(path.suffix, img)
                if not ok:
                    raise RuntimeError("Failed to encode output image")
                encoded.tofile(output_dir / path.name)
                saved += 1
                detected += bool(detections)
                print(f"[{index}/{len(images)}] {path.name}: {len(detections)} targets", flush=True)
            except Exception as exc:
                failed += 1
                print(f"[{index}/{len(images)}] FAILED {path.name}: {exc}", flush=True)
    print(f"Done: {saved} saved, {detected} images with detections, {failed} failed", flush=True)
    if failed:
        raise SystemExit(1)


if __name__ == "__main__":
    main()
