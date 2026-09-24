import argparse
import json
import os
import pickle
import subprocess
import threading
from contextlib import closing
from http.server import BaseHTTPRequestHandler, HTTPServer
from pathlib import Path

import cv2
import numpy as np

from image_writer import ImageWriter


SERVER_HOST = "0.0.0.0"
SERVER_PORT = 8790
CONDA_ENVS = Path.home() / "anaconda3/envs"


class ModelProcess:
    """Run a model server in its conda environment over local binary pipes."""

    def __init__(
        self,
        name,
        init_args=(),
    ):
        python = CONDA_ENVS / name / "python.exe"
        script = {"sam3": "detector_sam3.py", "da3": "depth_da3.py"}[name]
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
        self.reply(200, {"ok": True}) if self.path == "/health" else self.reply(404, {"error": "not found"})

    def do_POST(self):
        if self.path != "/detect":
            self.reply(404, {"error": "not found"})
            return
        img = None
        annotations = []
        try:
            self.connection.settimeout(30)
            length = int(self.headers.get("Content-Length", "0"))
            if not 0 < length <= 64 * 1024 * 1024:
                raise ValueError("Request size must be between 1 byte and 64 MiB")
            img = decode_img(self.rfile.read(length))
            pose = json.loads(self.headers["X-Pose"])
            detections = self.server.detector.request((img,))
            annotations = [(item["box"], item["confidence"]) for item in detections]
            if detections:
                locations = self.server.depth.request((
                    img,
                    [item.pop("mask") for item in detections],
                    pose,
                ))
                detections = [{**item, **location, "world_frame": pose["frame_id"]}
                              for item, location in zip(detections, locations)]
        except Exception as exc:
            self.reply(500, {"error": str(exc)})
        else:
            self.reply(200, detections)
        finally:
            # Queue after replying; the worker owns drawing, encoding and disk I/O.
            if img is not None:
                self.server.image_writer.submit(img, annotations)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
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
    args = parser.parse_args()
    try:
        reference_img, reference_box = load_reference(args.ref_img, args.ref_box)
    except Exception as exc:
        parser.error(str(exc))

    # Each child loads and warms up its model before accepting requests.
    with HTTPServer((SERVER_HOST, SERVER_PORT), Handler) as server:
        with closing(ModelProcess("sam3", (reference_img, reference_box, (1080, 1920)))) as detector:
            with closing(ModelProcess("da3")) as depth, closing(ImageWriter()) as image_writer:
                server.detector, server.depth = detector, depth
                server.image_writer = image_writer
                print(f"Ready: http://{SERVER_HOST}:{SERVER_PORT}", flush=True)
                try:
                    server.serve_forever()
                except KeyboardInterrupt:
                    pass


if __name__ == "__main__":
    main()
