import atexit
import json
import math
import secrets
import socket
import struct
import threading
import time
import urllib.request

import cv2


RTSP_URL = "rtsp://192.168.144.64:558/live/single"
JPEG_QUALITY = 95
WINDOWS_IP = "192.168.31.66"
DETECTOR_PORT = 8790
STREAM_OPEN_TIMEOUT_S = 3.0
STREAM_READ_TIMEOUT_S = 1.0
FIRST_FRAME_TIMEOUT_S = 5.0
FRAME_WAIT_TIMEOUT_S = 0.5
CAMERA_ADDRESS = ("192.168.144.64", 1030)
GIMBAL_TIMEOUT_S = 5.0
GIMBAL_TOLERANCE_DEG = 0.5


_worker = None
_frame = None
_error = None
_lock = threading.Lock()
_ready = threading.Event()
_stop = threading.Event()


def _read_stream():
    """Own the RTSP connection and replace the cached frame continuously."""
    global _frame, _error
    cap = cv2.VideoCapture()
    try:
        if not cap.open(
            RTSP_URL,
            cv2.CAP_FFMPEG,
            [cv2.CAP_PROP_OPEN_TIMEOUT_MSEC, int(STREAM_OPEN_TIMEOUT_S * 1000),
             cv2.CAP_PROP_READ_TIMEOUT_MSEC, int(STREAM_READ_TIMEOUT_S * 1000)],
        ):
            raise RuntimeError("Failed to open K40T stream")
        while not _stop.is_set():
            ok, img = cap.read()
            if (
                not ok
                or img is None
            ):
                raise RuntimeError("Failed to read K40T stream")
            _frame = img, time.monotonic()
            _ready.set()
    except Exception as exc:
        _error = exc
        _ready.set()
    finally:
        cap.release()


def get_img():
    """Wait for a new frame; return its image and monotonic receipt time."""
    global _worker, _frame, _error
    with _lock:
        _ready.clear()
        if (
            _worker is None
            or not _worker.is_alive()
        ):
            _frame = _error = None
            _stop.clear()
            _worker = threading.Thread(target=_read_stream, daemon=True)
            _worker.start()
        timeout = FIRST_FRAME_TIMEOUT_S if _frame is None else FRAME_WAIT_TIMEOUT_S
        if (
            _error is None
            and not _ready.wait(timeout=timeout)
        ):
            raise RuntimeError("Timed out waiting for K40T image")
        if _error is not None:
            raise RuntimeError("K40T stream failed") from _error
        if _stop.is_set():
            raise RuntimeError("K40T stream is closing")
        img, received = _frame
        return img.copy(), received  # img: BGR numpy.ndarray, shape=(H, W, 3), dtype=uint8.


def close_camera():
    """Stop the reader and release the shared RTSP connection."""
    global _worker, _frame
    with _lock:
        if _worker is not None:
            _stop.set()
            _worker.join(timeout=5)
            if _worker.is_alive():
                raise RuntimeError("K40T stream did not close in time")
            _worker = _frame = None


atexit.register(close_camera)


def encode_img(img):
    """Encode a BGR image as JPEG bytes."""
    ok, encoded = cv2.imencode(
        ".jpg",
        img,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("Can not encode K40T img!!!")
    return encoded.tobytes()


def detect_img(img, pose):
    """Send a JPEG image and its paired pose; return detections with 3D positions."""
    request = urllib.request.Request(
        f"http://{WINDOWS_IP}:{DETECTOR_PORT}/detect",
        encode_img(img),
        {"Content-Type": "image/jpeg", "X-Pose": json.dumps(pose, allow_nan=False)},
    )
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open(request, timeout=120) as response:
        return json.load(response)


def check_tracker():
    """Require a running tracker that has completed model warmup."""
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    try:
        with http.open(f"http://{WINDOWS_IP}:{DETECTOR_PORT}/health", timeout=120) as response:
            status = json.load(response)
    except Exception as exc:
        raise RuntimeError(f"Failed to confirm tracker readiness: {exc}") from exc
    if status.get("tracker_ready") is not True:
        raise RuntimeError("Tracker is unavailable; start the Windows server with --autofocus and wait for Ready")


def track_img(img, box=None):
    """Initialize with a current-image box, or track the next BGR frame."""
    headers = {"Content-Type": "image/jpeg"}
    path = "/track"
    if box is not None:
        headers["X-Box"] = json.dumps(box, allow_nan=False)
        path = "/track/init"
    request = urllib.request.Request(
        f"http://{WINDOWS_IP}:{DETECTOR_PORT}{path}",
        encode_img(img),
        headers,
    )
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open(request, timeout=120) as response:
        return json.load(response)


def crc16(data):
    crc = 0xFFFF
    for byte in data:
        value = byte ^ (crc & 0xFF)
        value = (value ^ (value << 4)) & 0xFF
        crc = ((crc >> 8) ^ (value << 8) ^ (value << 3) ^ (value >> 4)) & 0xFFFF
    return crc


def camera_request(
    message_id,
    payload,
    target=None,
    guard=None,
    mount=None,
    kind="gimbal",
):
    sequence = secrets.randbelow(256)
    body = bytes((len(payload), 4, 1, sequence, 1, 1))
    body += message_id.to_bytes(3, "little") + payload
    packet = b"\xfd" + body + struct.pack("<H", crc16(body))
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        sock.connect(CAMERA_ADDRESS)
        sock.settimeout(0.2)
        if guard is not None:
            guard()
        sock.send(packet)
        deadline = time.monotonic() + GIMBAL_TIMEOUT_S
        acknowledged = False
        stable = 0
        while time.monotonic() < deadline:
            if guard is not None:
                guard()
            try:
                raw = sock.recv(4096)
            except socket.timeout:
                continue
            if (
                len(raw) < 12
                or raw[0] != 0xFD
                or len(raw) != raw[1] + 12
                or raw[2:4] != b"\x01\x01"
                or raw[5:7] != b"\x04\x01"
                or crc16(raw[1:-2]) != int.from_bytes(raw[-2:], "little")
            ):
                continue
            reply_id = int.from_bytes(raw[7:10], "little")
            data = raw[10:-2]
            if (
                reply_id == message_id | 0x010000
                and raw[4] == sequence
                and len(data) >= 2
            ):
                if data[:2] != b"\x00\x00":
                    raise RuntimeError("K40T rejected camera command")
                acknowledged = True
            elif (
                acknowledged
                and kind == "zoom"
                and reply_id in (0x000005, 0x020005)
                and len(data) >= 15
            ):
                moving, _, tenths = struct.unpack_from("<BHH", data)
                if (
                    moving not in (0, 1)
                    or not 10 <= tenths <= 1600
                ):
                    raise RuntimeError("Invalid K40T zoom status")
                if target is None:
                    return tenths / 10.0
                if (
                    moving == 0
                    and tenths == target
                ):
                    return tenths / 10.0
            elif (
                acknowledged
                and kind == "gimbal"
                and reply_id in (1, 0x020001)
                and len(data) >= 7
            ):
                if (
                    data[0] & 0x0F
                    or data[4] not in (0, 1)
                ):
                    raise RuntimeError("Invalid K40T gimbal status")
                if (
                    mount is not None
                    and mount != data[4]
                ):
                    raise RuntimeError("K40T mounting orientation changed")
                mount = data[4]
            elif (
                acknowledged
                and kind == "gimbal"
                and mount is not None
                and reply_id in (2, 0x020002)
                and len(data) >= 20
            ):
                yaw, _, pitch = struct.unpack_from("<hhh", data)
                if (
                    abs(yaw) > 18000
                    or abs(pitch) > 18000
                ):
                    raise RuntimeError("Invalid K40T joint angles")
                yaw, pitch = yaw / 100.0, pitch / 100.0
                if mount == 1:
                    pitch = 180 - pitch if pitch > 0 else -180 - pitch
                status = {"pitch_deg": pitch, "yaw_deg": yaw, "mount": mount}
                if target is None:
                    return status
                reached = all(abs(status[key] - target[key]) <= GIMBAL_TOLERANCE_DEG for key in target)
                stable = stable + 1 if reached else 0
                if stable >= 3:
                    return status
        # Never resend a movement whose outcome is unknown.
        raise RuntimeError(f"K40T {kind} confirmation timed out")


def get_gimbal(guard=None):
    return camera_request(
        0x000200,
        b"\x01\x00",
        guard=guard,
    )


def set_gimbal(
    pitch_deg,
    yaw_deg,
    guard=None,
    *,
    mount=None,
):
    """Set absolute joint angles: pitch positive up, yaw positive right."""
    if (
        not math.isfinite(pitch_deg)
        or not math.isfinite(yaw_deg)
        or not -90 <= pitch_deg <= 30
        or not -180 <= yaw_deg <= 180
    ):
        raise ValueError("Gimbal target is outside its angular limits")
    # Relative moves reuse the mounting status from their fresh angle query.
    if mount is None:
        mount = get_gimbal(guard)["mount"]
    payload = struct.pack(
        "<BHBHB",
        0 if pitch_deg >= 0 else 1,
        round(abs(pitch_deg) * 100),
        1 if yaw_deg >= 0 else 0,
        round(abs(yaw_deg) * 100),
        0,
    )
    return camera_request(
        0x12,
        payload,
        {"pitch_deg": pitch_deg, "yaw_deg": yaw_deg},
        guard,
        mount,
    )


def gimbal_pitch(delta_deg, guard=None):
    status = get_gimbal(guard)
    return set_gimbal(
        status["pitch_deg"] + delta_deg,
        status["yaw_deg"],
        guard,
        mount=status["mount"],
    )


def gimbal_yaw(delta_deg, guard=None):
    status = get_gimbal(guard)
    return set_gimbal(
        status["pitch_deg"],
        status["yaw_deg"] + delta_deg,
        guard,
        mount=status["mount"],
    )


def get_zoom(guard=None):
    return camera_request(
        0x000200,
        b"\x01\x00",
        guard=guard,
        kind="zoom",
    )


def set_zoom(ratio, guard=None):
    """Set absolute zoom; wait for measured magnification and motor completion."""
    if (
        not math.isfinite(ratio)
        or not 1 <= ratio <= 160
    ):
        raise ValueError("Zoom must be between 1 and 160")
    tenths = round(ratio * 10)
    return camera_request(
        0x000304,
        struct.pack(
            "<BH",
            0,
            tenths,
        ),
        target=tenths,
        guard=guard,
        kind="zoom",
    )
