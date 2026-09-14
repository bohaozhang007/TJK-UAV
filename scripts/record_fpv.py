"""Record the camera's RTSP stream locally; Ctrl+C finalizes the recording."""
import argparse
import datetime as dt
import os
from pathlib import Path
import shutil
import signal
import subprocess
import time


def find_ffmpeg():
    found = shutil.which("ffmpeg")
    if found:
        return found
    binaries = Path.home()/"anaconda3/envs/da3/Lib/site-packages/imageio_ffmpeg/binaries"
    return next((str(path) for path in binaries.glob("ffmpeg*.exe")), None)


def record(command, log_path):
    stopping = False

    def stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signals = [signal.SIGINT]
    if hasattr(signal, "SIGBREAK"):
        signals.append(signal.SIGBREAK)
    previous = {sig: signal.signal(sig, stop) for sig in signals}
    process = None
    try:
        with log_path.open("w", encoding="utf-8") as log:
            # Prevent console Ctrl+C from terminating FFmpeg before it closes MKV.
            options = ({"creationflags": subprocess.CREATE_NEW_PROCESS_GROUP}
                       if os.name == "nt" else {"start_new_session": True})
            process = subprocess.Popen(command, stdin=subprocess.PIPE,
                                       stdout=log, stderr=log, **options)
            while process.poll() is None and not stopping:
                time.sleep(0.1)
            if process.poll() is None:
                print("Stopping recording; saving file...", flush=True)
                try:
                    process.communicate(input=b"q\n", timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.communicate()
                    print("FFmpeg did not finish within 15 s; recording may be incomplete.")
                    return 1
            return process.returncode
    finally:
        if process is not None:
            if process.poll() is None:
                process.kill()
                process.wait()
            if process.stdin:
                process.stdin.close()
        for sig, handler in previous.items():
            signal.signal(sig, handler)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--url", default="rtsp://192.168.2.20:8554/live/0")
    parser.add_argument("--output-dir", type=Path,
                        default=Path.home()/"Documents/QGroundControl/Video")
    parser.add_argument("--ffmpeg", default=find_ffmpeg())
    args = parser.parse_args()
    if not args.ffmpeg:
        parser.error("FFmpeg not found; provide --ffmpeg PATH")
    args.output_dir.mkdir(parents=True, exist_ok=True)
    stem = "fpv_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f")
    output = args.output_dir/(stem + ".mkv")
    log = args.output_dir/(stem + ".log")
    command = [args.ffmpeg, "-hide_banner", "-nostats", "-n",
               "-rtsp_transport", "tcp", "-timeout", "5000000", "-i", args.url,
               "-map", "0:v:0", "-c:v", "copy", "-an", str(output)]
    print(f"Recording to: {output}\nPress Ctrl+C to stop and save.", flush=True)
    try:
        code = record(command, log)
    except OSError as exc:
        print(f"Cannot record: {exc}")
        return 1
    if code == 0 and output.exists() and output.stat().st_size:
        print(f"Saved: {output}")
        return 0
    print(f"Recording failed (exit {code}); inspect: {log}")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
