"""Check RTSP decoding without ROS, flight control, or inference services."""

import argparse
from pathlib import Path
import subprocess
import sys
import time


sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from hardware.video_stream import RTSP_URL, cv2, open_stream


DEFAULT_FRAMES = 10
DEFAULT_TIMEOUT_S = 15.0


def check_camera(args):
    print(f"OpenCV: {cv2.__version__}; stream: {RTSP_URL}", flush=True)
    started = time.monotonic()
    cap = open_stream()
    try:
        for index in range(args.frames):
            ok, img = cap.read()
            if (
                not ok
                or img is None
                or img.size == 0
            ):
                raise RuntimeError(f"Failed to decode frame {index + 1}/{args.frames}")
            height, width = img.shape[:2]
            print(f"Frame {index + 1}/{args.frames}: {width}x{height}", flush=True)
        if args.save:
            if not cv2.imwrite(args.save, img):
                raise RuntimeError(f"Failed to save image: {args.save}")
            print(f"Saved: {args.save}", flush=True)
    finally:
        cap.release()
    print(f"PASS: decoded {args.frames} frames in {time.monotonic() - started:.2f}s", flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--frames", type=int, default=DEFAULT_FRAMES)
    parser.add_argument("--timeout", type=float, default=DEFAULT_TIMEOUT_S)
    parser.add_argument("--save", help="Optional output image path, e.g. /tmp/k40t.jpg")
    parser.add_argument("--worker", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args()
    if (
        args.frames <= 0
        or args.timeout <= 0
    ):
        parser.error("--frames and --timeout must be positive")
    if args.worker:
        try:
            check_camera(args)
        except Exception as exc:
            print(f"FAIL: {type(exc).__name__}: {exc}", file=sys.stderr, flush=True)
            return 1
        return 0
    # A subprocess also bounds native FFmpeg calls and cap.release().
    try:
        result = subprocess.run(
            [sys.executable, str(Path(__file__).resolve()), *sys.argv[1:], "--worker"],
            timeout=args.timeout,
        )
        return result.returncode
    except subprocess.TimeoutExpired:
        print(f"FAIL: camera check exceeded {args.timeout:g}s", file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(main())
