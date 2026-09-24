import atexit
import json
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


def send_img(img, pose):
    """Send a JPEG image and its paired pose; return detections with 3D positions."""
    request = urllib.request.Request(
        f"http://{WINDOWS_IP}:{DETECTOR_PORT}/detect",
        encode_img(img),
        {"Content-Type": "image/jpeg", "X-Pose": json.dumps(pose, allow_nan=False)},
    )
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open(request, timeout=120) as response:
        return json.load(response)
