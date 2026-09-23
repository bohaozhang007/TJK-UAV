import base64
import json
import urllib.request

import cv2


RTSP_URL = "rtsp://192.168.144.64:558/live/single"
JPEG_QUALITY = 95
WINDOWS_IP = "192.168.31.66"
DETECTOR_PORT = 8790


def get_img():
    cap = cv2.VideoCapture(RTSP_URL, cv2.CAP_FFMPEG)
    try:
        ok, img = cap.read()
        if not ok or img is None:
            raise RuntimeError("Can not read K40T img!!!")
        return img  # BGR, numpy.ndarray, shape=(H, W, 3)，dtype=uint8
    finally:
        cap.release()


def encode_img(img):
    """JPEG => Base64"""
    ok, encoded = cv2.imencode(
        ".jpg",
        img,
        [cv2.IMWRITE_JPEG_QUALITY, JPEG_QUALITY],
    )
    if not ok:
        raise RuntimeError("Can not encode K40T img!!!")
    return base64.b64encode(encoded).decode("ascii")


def send_img(img):
    """Send a JPEG-encoded BGR image to Windows and return detected xyxy boxes."""
    data = {"current_img": encode_img(img)}
    request = urllib.request.Request(
        f"http://{WINDOWS_IP}:{DETECTOR_PORT}/detect",
        json.dumps(data).encode("utf-8"),
        {"Content-Type": "application/json"},
    )
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open(request, timeout=120) as response:
        return json.load(response)
