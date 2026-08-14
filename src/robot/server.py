from __future__ import annotations

import argparse
import io
import json
import threading
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from typing import Any, Protocol
from urllib.parse import parse_qs, urlparse

import numpy as np

from .keyboard_op import KeyboardOp, VelocityController


class RobotController(VelocityController, Protocol):
    def init(self) -> dict[str, Any]: ...

    def takeoff(self) -> dict[str, Any]: ...

    def get_rgb_meta(self, save: bool = True) -> dict[str, Any]: ...

    def get_rgb_byte(self) -> bytes: ...

    def get_depth_meta(self, save: bool = True) -> dict[str, Any]: ...

    def get_depth_np(self) -> np.ndarray | dict[str, Any]: ...

    def move_relative_xyz(
        self,
        x: int,
        y: int,
        z: int,
    ) -> dict[str, Any]: ...

    def move_relative_xyz_yaw(
        self,
        x: int,
        y: int,
        z: int,
        yaw: int,
        timeout_s: float | None = None,
    ) -> dict[str, Any]: ...

    def rotate(self, angle_deg: int) -> dict[str, Any]: ...

    def get_pose(self) -> dict[str, Any]: ...

    def get_motion_tolerances(self) -> dict[str, Any]: ...

    def close(self) -> dict[str, Any]: ...


class Keepalive(Protocol):
    def start_keepalive(self) -> None: ...

    def stop_keepalive(self) -> None: ...

    def stop_thread(self) -> None: ...


class NullKeepalive:
    def start_keepalive(self) -> None:
        pass

    def stop_keepalive(self) -> None:
        pass

    def stop_thread(self) -> None:
        pass


class KeepaliveThread(threading.Thread):
    """后台心跳线程，每10秒发送一次 velocity(0,0,0,0) 以保持连接"""
    def __init__(self, controller: VelocityController):
        super().__init__(daemon=True)
        self.controller = controller
        self._stop_event = threading.Event()
        self._active = False

    def start_keepalive(self):
        self._active = True

    def stop_keepalive(self):
        self._active = False

    def stop_thread(self):
        self._stop_event.set()

    def run(self):
        while not self._stop_event.is_set():
            # 阻塞等待 10 秒，如果中途触发了 stop_event 则立刻唤醒并退出
            is_stopped = self._stop_event.wait(10.0)
            if is_stopped:
                break
            
            if self._active:
                try:
                    self.controller.velocity(0, 0, 0, 0)
                except Exception:
                    # 忽略心跳包引发的异常，避免污染控制台输出
                    pass


class ApiHandler(BaseHTTPRequestHandler):
    controller: RobotController | None = None
    keepalive: Keepalive | None = None
    keyboard_op: KeyboardOp | None = None

    def _json_response(self, code: int, payload: dict):
        body = json.dumps(payload, ensure_ascii=False).encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "application/json; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def _html_response(self, code: int, html: str):
        body = html.encode("utf-8")
        self.send_response(code)
        self.send_header("Content-Type", "text/html; charset=utf-8")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.end_headers()
        self.wfile.write(body)

    def _bytes_response(self, code: int, content_type: str, body: bytes):
        self.send_response(code)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.end_headers()
        self.wfile.write(body)

    def _npy_response(self, code: int, array: np.ndarray):
        if not isinstance(array, np.ndarray):
            raise TypeError("get_depth_np must return a numpy.ndarray or a result dict")
        buffer = io.BytesIO()
        np.save(buffer, array, allow_pickle=False)
        self._bytes_response(code, "application/x-npy", buffer.getvalue())

    @staticmethod
    def _preview_html() -> str:
        return """<!doctype html>
<html lang="en">
  <head>
    <meta charset="utf-8">
    <title>Drone Live Preview</title>
    <style>
      body { font-family: Arial, sans-serif; margin: 24px; background: #111; color: #eee; }
      h1 { margin-bottom: 8px; }
      p { color: #bbb; }
      img { max-width: min(96vw, 960px); width: 100%; border: 1px solid #444; background: #000; }
      a { color: #8ecbff; }
      code { color: #f3d17a; }
    </style>
  </head>
  <body>
    <h1>Drone Live Preview</h1>
    <p>Run <code>init</code> first, then this page will show the live MJPEG stream.</p>
    <p><a href="/get_rgb_byte" target="_blank" rel="noreferrer">Open single-frame snapshot</a></p>
    <img src="/video_feed" alt="Drone live video stream" />
  </body>
</html>
"""

    def _stream_video_feed(self, fps: float = 10.0):
        assert self.controller is not None
        boundary = "frame"
        interval_seconds = 1.0 / max(1.0, min(30.0, float(fps)))
        frame_bytes = self.controller.get_rgb_byte()

        self.send_response(200)
        self.send_header("Content-Type", f"multipart/x-mixed-replace; boundary={boundary}")
        self.send_header("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0")
        self.send_header("Pragma", "no-cache")
        self.send_header("Connection", "close")
        self.end_headers()

        while True:
            try:
                self.wfile.write(f"--{boundary}\r\n".encode("ascii"))
                self.wfile.write(b"Content-Type: image/jpeg\r\n")
                self.wfile.write(f"Content-Length: {len(frame_bytes)}\r\n\r\n".encode("ascii"))
                self.wfile.write(frame_bytes)
                self.wfile.write(b"\r\n")
                self.wfile.flush()
                time.sleep(interval_seconds)
                frame_bytes = self.controller.get_rgb_byte()
            except (BrokenPipeError, ConnectionResetError, ConnectionAbortedError, OSError):
                return
            except Exception:
                return

    def _read_json_body(self) -> dict:
        content_length = int(self.headers.get("Content-Length", "0"))
        if content_length <= 0:
            return {}
        raw = self.rfile.read(content_length)
        try:
            return json.loads(raw.decode("utf-8"))
        except Exception:
            return {}

    @staticmethod
    def _request_params(query: dict, body: dict) -> dict:
        if not isinstance(body, dict):
            raise ValueError("JSON body must be an object")

        params = dict(body)
        for name, values in query.items():
            if values:
                params[name] = values[-1]
        return params

    def _handle(self):
        assert self.controller is not None
        assert self.keepalive is not None
        parsed = urlparse(self.path)
        path = parsed.path.rstrip("/") or "/"
        query = parse_qs(parsed.query)
        body = self._read_json_body() if self.command == "POST" else {}
        params = self._request_params(query, body)

        if path == "/preview":
            self._html_response(200, self._preview_html())
            return

        if path == "/get_rgb_byte":
            try:
                self._bytes_response(200, "image/jpeg", self.controller.get_rgb_byte())
            except Exception as exc:
                self._json_response(400, {"ok": False, "error": str(exc)})
            return

        if path == "/get_depth_np":
            try:
                result = self.controller.get_depth_np()
                if isinstance(result, dict):
                    self._json_response(200, result)
                else:
                    self._npy_response(200, result)
            except Exception as exc:
                self._json_response(400, {"ok": False, "error": str(exc)})
            return

        if path == "/video_feed":
            try:
                fps = float(query.get("fps", ["10"])[0])
                self._stream_video_feed(fps=fps)
            except Exception as exc:
                self._json_response(400, {"ok": False, "error": str(exc)})
            return

        try:
            if (
                self.keyboard_op is not None
                and self.keyboard_op.is_active()
                and path
                not in {
                    "/health",
                    "/get_pose",
                    "/get_rgb_meta",
                    "/get_depth_meta",
                    "/motion_tolerances",
                }
            ):
                self._json_response(
                    409,
                    {"ok": False, "error": "keyboard operation is active"},
                )
                return

            if path == "/init":
                result = self.controller.init()
                self.keepalive.start_keepalive()  # 激活心跳
            elif path == "/takeoff":
                result = self.controller.takeoff()
            elif path == "/get_rgb_meta":
                result = self.controller.get_rgb_meta(**params)
            elif path == "/get_depth_meta":
                result = self.controller.get_depth_meta(**params)
            elif path == "/velocity":
                result = self.controller.velocity(**params)
            elif path == "/move_relative_xyz":
                result = self.controller.move_relative_xyz(**params)
            elif path == "/move_relative_xyz_yaw":
                move_relative_xyz_yaw = getattr(
                    self.controller,
                    "move_relative_xyz_yaw",
                    None,
                )
                if move_relative_xyz_yaw is None:
                    raise NotImplementedError(
                        "move_relative_xyz_yaw is unsupported by this controller"
                    )
                result = move_relative_xyz_yaw(**params)
            elif path == "/rotate":
                result = self.controller.rotate(**params)
            elif path == "/get_pose":
                result = self.controller.get_pose()
            elif path == "/motion_tolerances":
                result = {
                    "ok": True,
                    "motion_tolerances": self.controller.get_motion_tolerances(),
                }
            elif path == "/land":
                result = self.controller.land()
            elif path == "/close":
                self.keepalive.stop_keepalive()
                result = self.controller.close()
            elif path == "/health":
                result = {"ok": True, "health": self.controller.health()}
            else:
                self._json_response(404, {"ok": False, "error": f"unknown path: {path}"})
                return
            self._json_response(200, result)
        except Exception as exc:
            self._json_response(400, {"ok": False, "error": str(exc)})

    def do_GET(self):
        self._handle()

    def do_POST(self):
        self._handle()


def run_http_server(
    controller: RobotController,
    keepalive: Keepalive,
    host: str,
    port: int,
    keyboard_op: KeyboardOp | None = None,
):
    ApiHandler.controller = controller
    ApiHandler.keepalive = keepalive
    ApiHandler.keyboard_op = keyboard_op
    server = ThreadingHTTPServer((host, port), ApiHandler)
    thread = threading.Thread(target=server.serve_forever, daemon=True)
    thread.start()
    return server


def run_console(
    controller: RobotController,
    keepalive: Keepalive,
    keyboard_op: KeyboardOp | None = None,
):
    if keyboard_op is None:
        keyboard_op = KeyboardOp(controller)
    print(
        "Headless console ready. Commands: "
        "init | takeoff | k | get_rgb_meta | get_depth_meta | get_pose | "
        "move_rel_xyz X Y Z | move_rel_xyz_yaw X Y Z YAW | "
        "rotate YAW | health | abort | land | force_land | close | quit"
    )
    while True:
        try:
            raw = input("> ").strip()
        except (KeyboardInterrupt, EOFError):
            raw = "quit"

        if not raw:
            continue

        parts = raw.split()
        cmd = parts[0].lower()
        try:
            if cmd in {"quit", "exit"}:
                keepalive.stop_keepalive()
                print(controller.close())
                print("bye")
                break
            if cmd == "init":
                console_init = getattr(controller, "console_init", None)
                res = (
                    console_init()
                    if console_init is not None
                    else controller.init()
                )
                keepalive.start_keepalive() # 激活心跳
                print(res)
            elif cmd == "takeoff":
                print(controller.takeoff())
            elif cmd == "k":
                health = controller.health()
                if health.get("velocity_control_supported") is False:
                    print(
                        {
                            "ok": False,
                            "error": "keyboard velocity control is unsupported",
                        }
                    )
                    continue
                if not health.get("initialized"):
                    print({"ok": False, "error": "call init before keyboard control"})
                    continue
                if not health.get("airborne"):
                    print({"ok": False, "error": "call takeoff before keyboard control"})
                    continue

                keepalive.stop_keepalive()
                print(
                    "Keyboard control: T/G forward/back, F/H left/right, "
                    "I/K up/down, J/L yaw, Space hover, B land, Esc return."
                )
                try:
                    print(keyboard_op.run_foreground())
                finally:
                    if controller.health().get("initialized"):
                        keepalive.start_keepalive()
            elif cmd == "get_rgb_meta":
                print(controller.get_rgb_meta(save=True))
            elif cmd == "get_depth_meta":
                print(controller.get_depth_meta(save=True))
            elif cmd == "get_pose":
                print(controller.get_pose())
            elif cmd in {"move_rel_xyz", "move_relative_xyz"}:
                if len(parts) != 4:
                    raise ValueError("usage: move_rel_xyz X_CM Y_CM Z_CM")
                x, y, z = (int(value) for value in parts[1:])
                print(controller.move_relative_xyz(x=x, y=y, z=z))
            elif cmd in {"move_rel_xyz_yaw", "move_relative_xyz_yaw"}:
                if len(parts) != 5:
                    raise ValueError(
                        "usage: move_rel_xyz_yaw X_CM Y_CM Z_CM YAW_DEG"
                    )
                x, y, z, yaw = (int(value) for value in parts[1:])
                print(
                    controller.move_relative_xyz_yaw(
                        x=x,
                        y=y,
                        z=z,
                        yaw=yaw,
                    )
                )
            elif cmd == "rotate":
                if len(parts) != 2:
                    raise ValueError("usage: rotate YAW_DEG")
                print(controller.rotate(angle_deg=int(parts[1])))
            elif cmd == "health":
                print({"ok": True, "health": controller.health()})
            elif cmd == "abort":
                abort = getattr(controller, "abort", None)
                if abort is None:
                    raise NotImplementedError(
                        "abort is unsupported by this controller"
                    )
                print(abort())
            elif cmd == "land":
                print(controller.land())
            elif cmd == "force_land":
                force_land = getattr(controller, "force_land", None)
                if force_land is None:
                    raise NotImplementedError(
                        "force_land is unsupported by this controller"
                    )
                print(force_land())
            elif cmd == "close":
                keepalive.stop_keepalive()
                print(controller.close())
            else:
                print("unknown command")
        except Exception as exc:
            print({"ok": False, "error": str(exc)})


def build_controller(robot: str, image_dir: str) -> RobotController:
    if robot == "tello":
        from .controllers.tello import TelloController

        return TelloController(image_dir=image_dir)
    if robot == "ue":
        from .controllers.ue import UEController

        return UEController(image_dir=image_dir)
    if robot == "owl":
        from .controllers.owl import OwlController

        return OwlController(image_dir=image_dir)
    if robot == "i7":
        from .controllers.i7 import I7Controller

        return I7Controller(image_dir=image_dir)
    raise ValueError(f"unsupported robot: {robot}")


def build_keep_alive(robot: str, controller: RobotController) -> Keepalive:
    if robot == "tello":
        keepalive = KeepaliveThread(controller)
        keepalive.start()
        return keepalive
    if robot in {"ue", "owl", "i7"}:
        return NullKeepalive()
    raise ValueError(f"unsupported robot: {robot}")


def _build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Robot Controller")
    parser.add_argument(
        "--robot",
        choices=("tello", "ue", "owl", "i7"),
        default="ue",
        help="controller backend to use (default: ue)",
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument(
        "--vdir",
        default=str(Path(__file__).resolve().parents[2] / "captures"),
        help="Directory for captured RGB images and depth data",
    )
    return parser


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    controller = build_controller(args.robot, args.vdir)
    keepalive = build_keep_alive(args.robot, controller)
    keyboard_op = KeyboardOp(controller)

    server = run_http_server(
        controller,
        keepalive,
        args.host,
        args.port,
        keyboard_op=keyboard_op,
    )

    print(f"HTTP ready at http://{args.host}:{args.port} robot={args.robot}")
    print(f"Live preview: http://{args.host}:{args.port}/preview")
    print(
        "Endpoints: /init /takeoff /get_rgb_meta /get_rgb_byte "
        "/get_depth_meta /get_depth_np /velocity /move_relative_xyz "
        "/move_relative_xyz_yaw /rotate /get_pose /motion_tolerances "
        "/land /close /health /video_feed /preview"
    )

    try:
        run_console(controller, keepalive, keyboard_op)
    finally:
        keepalive.stop_keepalive()
        try:
            close_result = controller.close()
            if not close_result.get("ok", False):
                print(
                    {
                        "ok": False,
                        "error": "controller close failed",
                        "result": close_result,
                    }
                )
        except Exception as exc:
            print({"ok": False, "error": f"controller close failed: {exc}"})
        finally:
            keepalive.stop_thread() # 安全退出心跳线程
            server.shutdown()
            server.server_close()


if __name__ == "__main__":
    main()
