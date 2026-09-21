"""Short-lived encoded frames shared with the local depth service."""
from collections import OrderedDict
import time
import uuid


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
