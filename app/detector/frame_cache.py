"""Short-lived encoded frames shared with the local depth service."""
from collections import OrderedDict
import time
import uuid
import urllib.request


def cached_image(token, frame_id):
    if not isinstance(token,str) or len(token)!=32 or any(c not in '0123456789abcdef' for c in token):
        raise ValueError('invalid image cache token')
    http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
    with http.open('http://127.0.0.1:8790/frames/'+token,timeout=3.) as response:
        if response.headers.get('X-Frame-Id') != frame_id:
            raise ValueError('cached image frame mismatch')
        image = response.read(32*1024*1024+1)
        if len(image)>32*1024*1024:
            raise ValueError('cached image exceeds size limit')
        return image


class FrameCache:
    def __init__(self):
        self.frames = OrderedDict()

    def expire(self):
        now = time.monotonic()
        for token, entry in list(self.frames.items()):
            if now-entry[0] > 120.:
                del self.frames[token]

    def put(self, frame_id, encoded):
        self.expire()
        token = uuid.uuid4().hex
        self.frames[token] = (time.monotonic(), frame_id, encoded)
        while len(self.frames) > 4:
            self.frames.popitem(last=False)
        return token

    def get(self, token):
        self.expire()
        entry = self.frames.get(token)
        return (entry[1], entry[2]) if entry else None
