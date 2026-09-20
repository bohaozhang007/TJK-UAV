"""Client for app/tracker/server.py; one init followed by stateful track calls."""

from __future__ import annotations

import base64
import json
import math
from urllib.error import HTTPError, URLError
from urllib.parse import urlparse
from urllib.request import ProxyHandler, Request, build_opener

import cv2
import numpy as np


def validate_image_box(img, box):
    if not isinstance(img, np.ndarray) or img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("img must be a uint8 HxWx3 BGR image")
    height, width = img.shape[:2]
    if width * height > 8_000_000:
        raise ValueError("Tracker accepts at most 8,000,000 pixels; image will not be resized")
    bounds = np.asarray(box, dtype=float)
    if bounds.shape != (4,) or not np.isfinite(bounds).all():
        raise ValueError("box must contain four finite pixel coordinates: x1,y1,x2,y2")
    x1, y1, x2, y2 = bounds
    if not (0 <= x1 < x2 <= width and 0 <= y1 < y2 <= height):
        raise ValueError("box must lie inside img")
    return bounds


class TrackerClient:
    def __init__(self, url: str, timeout_s: float = 30):
        parsed = urlparse(url)
        if parsed.scheme not in ('http', 'https') or not parsed.hostname or parsed.query or parsed.fragment:
            raise ValueError('tracker_url must be an HTTP(S) base URL')
        if not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError('tracker timeout must be positive')
        self.url = url.rstrip('/')
        self.timeout_s = timeout_s
        self.shape = None
        # LAN tracker requests must not be sent through shell HTTP proxies.
        self._http = build_opener(ProxyHandler({}))

    def init(self, img, box, *, timeout_s=None):
        self.shape = None
        box = validate_image_box(img, box)
        result = self._post('/init', img, box.tolist(), timeout_s)
        self.shape = img.shape[:2]
        return result

    def track(self, img, *, timeout_s=None):
        if self.shape is None:
            raise RuntimeError('Tracker is not initialized')
        if img.shape[:2] != self.shape:
            raise RuntimeError('Live image dimensions differ from img; pass a native-resolution image')
        try:
            return self._post('/track', img, None, timeout_s)
        except BaseException:
            self.shape = None
            raise

    def _post(self, path, img, box, timeout_s):
        if not isinstance(img, np.ndarray) or img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
            raise ValueError('Tracker image must be uint8 BGR')
        # Encode full-size BGR losslessly. The server decodes PNG into RGB.
        ok, encoded = cv2.imencode('.png', img, [cv2.IMWRITE_PNG_COMPRESSION, 1])
        if not ok:
            raise RuntimeError('Cannot encode tracker image')
        payload = dict(image=base64.b64encode(encoded).decode('ascii'))
        if box is not None:
            payload['box'] = box
        body = json.dumps(payload, allow_nan=False).encode('utf-8')
        if len(body) > 32 * 1024 * 1024:
            raise ValueError('Tracker request exceeds 32 MiB')
        request = Request(self.url + path, data=body, headers={'Content-Type': 'application/json'})
        timeout = self.timeout_s if timeout_s is None else min(self.timeout_s, timeout_s)
        if timeout <= 0:
            raise RuntimeError('Tracker request deadline expired')
        try:
            with self._http.open(request, timeout=timeout) as response:
                data = response.read(1_048_577)
            if len(data) > 1_048_576:
                raise RuntimeError('Tracker response is too large')
            result = json.loads(data)
        except HTTPError as exc:
            detail = exc.read(2048).decode('utf-8', errors='replace')
            raise RuntimeError(f'Tracker {path} HTTP {exc.code}: {detail}') from exc
        except (URLError, OSError, ValueError) as exc:
            raise RuntimeError(f'Tracker {path} failed: {exc}') from exc
        if not isinstance(result, dict) or result.get('ok') is not True:
            raise RuntimeError(f'Tracker {path} failed: {result}')
        if result.get('image_size') != [img.shape[1], img.shape[0]]:
            raise RuntimeError('Tracker returned inconsistent image_size')
        if result.get('found') is not True or result.get('box') is None:
            raise RuntimeError(f'Target lost: tracker {path} returned found=false')
        return validate_image_box(img, result['box'])
