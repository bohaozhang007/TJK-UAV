import argparse
import json
import os
import pickle
import subprocess
import threading
from contextlib import ExitStack, closing
from datetime import datetime
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import numpy as np

from image_writer import ImageWriter


SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8790
CONDA_ENVS = Path.home() / "anaconda3/envs"
MODEL_SCRIPTS = {
    "sam3": "detector_sam3.py",
    "da3": "depth_da3.py",
    "sam2": "tracker_sam2.py",
}


class ModelProcess:
    """Run a model server in its conda environment over local binary pipes."""

    def __init__(
        self,
        name,
        init_args=(),
    ):
        python = CONDA_ENVS / name / "python.exe"
        script = MODEL_SCRIPTS[name]
        env = os.environ.copy()
        env["CONDA_PREFIX"] = str(python.parent)
        env["PYTHONIOENCODING"] = "utf-8"
        env["PATH"] = os.pathsep.join([
            str(python.parent),
            str(python.parent / "Library/bin"),
            str(python.parent / "Scripts"),
            env["PATH"],
        ])
        print(f"[{name}] Starting model process: {python}", flush=True)
        self.process = subprocess.Popen(
            [str(python), "-u", str(Path(__file__).with_name(script))],
            stdin=subprocess.PIPE,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            env=env,
            creationflags=subprocess.CREATE_NO_WINDOW,
        )
        self.log_worker = threading.Thread(target=self.forward_logs, daemon=True)
        self.log_worker.start()
        try:
            self.request(init_args)
        except BaseException:
            self.close()
            raise

    def forward_logs(self):
        # Drain logs independently; stdout remains reserved for binary model replies.
        for line in self.process.stderr:
            print(
                line.decode("utf-8", errors="replace"),
                end="",
                flush=True,
            )

    def receive(self):
        try:
            result, error = pickle.load(self.process.stdout)
        except EOFError as exc:
            raise RuntimeError("Model process exited unexpectedly") from exc
        if error:
            raise RuntimeError(error)
        return result

    def request(self, args):
        pickle.dump(args, self.process.stdin)
        self.process.stdin.flush()
        return self.receive()

    def close(self):
        self.process.stdin.close()
        try:
            self.process.wait(timeout=10)
        except subprocess.TimeoutExpired:
            self.process.terminate()
            self.process.wait()
        self.process.stdout.close()
        self.log_worker.join()
        self.process.stderr.close()

class Handler(BaseHTTPRequestHandler):
    def reply(
        self,
        status,
        data,
    ):
        body = json.dumps(data, allow_nan=False).encode()
        self.send_response(status)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if self.path != "/health":
            self.reply(404, {"error": "not found"})
            return
        self.reply(200, {"ok": True, "tracker_ready": self.tracker_ready()})

    def tracker_ready(self):
        # ModelProcess initialization returns only after successful warmup.
        return (
            self.server.tracker is not None
            and self.server.tracker.process.poll() is None
        )

    def do_POST(self):
        if self.path not in ("/detect", "/track/init", "/track"):
            self.reply(404, {"error": "not found"})
            return
        if (
            self.path != "/detect"
            and not self.tracker_ready()
        ):
            self.reply(503, {"error": "Tracker is unavailable; start the server with --autofocus"})
            return
        img = None
        annotations = []
        try:
            self.connection.settimeout(30)
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 64 * 1024 * 1024:
                raise ValueError("Request size must be between 1 byte and 64 MiB")
            img = decode_img(self.rfile.read(length))
            if self.path == "/detect":
                pose = json.loads(self.headers["X-Pose"])
                result = self.server.detector.request((img,))
                annotations = [item.copy() for item in result]
                if result:
                    locations = self.server.depth.request((
                        img,
                        [item.pop("mask") for item in result],
                        pose,
                    ))
                    result = [{**item, **location, "world_frame": pose["frame_id"]}
                              for item, location in zip(result, locations)]
            else:
                box = json.loads(self.headers["X-Box"]) if self.path == "/track/init" else None
                if (
                    self.path == "/track/init"
                    and box is None
                ):
                    raise ValueError("Tracker initialization requires a box")
                result = self.server.tracker.request((img, box))
                if result["box"] is not None:
                    annotations = [result.copy()]
                result.pop("mask")
        except Exception as exc:
            self.reply(500, {"error": str(exc)})
        else:
            self.reply(200, result)
        finally:
            # Queue after replying; the worker owns drawing, encoding and disk I/O.
            if img is not None:
                writer = self.server.detector_writer if self.path == "/detect" else self.server.tracker_writer
                writer.submit(img, annotations)


def decode_img(encoded):
    data = np.frombuffer(encoded, dtype=np.uint8)
    img = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to decode image")
    return img


def load_reference(img_path, box_path):
    img = cv2.imdecode(np.fromfile(img_path, dtype=np.uint8), cv2.IMREAD_COLOR)
    if img is None:
        raise ValueError("Failed to read reference image")
    box = np.array([float(v) for v in box_path.read_text(encoding="utf-8-sig").split()])
    h, w = img.shape[:2]
    if (
        box.shape != (4,)
        or not np.isfinite(box).all()
        or not 0 <= box[0] < box[2] <= w
        or not 0 <= box[1] < box[3] <= h
    ):
        raise ValueError("Reference box file must contain x1 y1 x2 y2 within the image")
    return img, box


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--ref-img",
        type=Path,
        required=True,
        help="Local reference image path",
    )
    parser.add_argument(
        "--ref-box",
        type=Path,
        required=True,
        help="Text file containing pixel x1 y1 x2 y2",
    )
    parser.add_argument(
        "--autofocus",
        action="store_true",
        help="Load and warm up SAM2 to enable autofocus tracking",
    )
    args = parser.parse_args()
    try:
        reference_img, reference_box = load_reference(args.ref_img, args.ref_box)
    except Exception as exc:
        parser.error(str(exc))

    # Each child loads and warms up its model before accepting requests.
    with ExitStack() as stack:
        server = stack.enter_context(HTTPServer((SERVER_HOST, SERVER_PORT), Handler))
        server.detector = stack.enter_context(closing(ModelProcess("sam3", (reference_img, reference_box, (1080, 1920)))))
        server.depth = stack.enter_context(closing(ModelProcess("da3")))
        server.tracker = None
        if args.autofocus:
            server.tracker = stack.enter_context(closing(ModelProcess("sam2")))
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")
        server.detector_writer = stack.enter_context(closing(ImageWriter("detector", timestamp)))
        if args.autofocus:
            server.tracker_writer = stack.enter_context(closing(ImageWriter("tracker", timestamp)))
        print(f"Ready: http://{SERVER_HOST}:{SERVER_PORT}", flush=True)
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            pass


if __name__ == "__main__":
    main()
