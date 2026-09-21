"""Bounded background encoding and storage of mission artifacts."""
from concurrent.futures import ThreadPoolExecutor
from copy import deepcopy
import json
from pathlib import Path
import sys
import threading
import time

from PIL import Image

REPORT_LOCK = threading.Lock()


def save_photo(path, rgb):
    Image.fromarray(rgb).save(path, quality=95)


class ArtifactWriter:
    def __init__(self):
        self.executor = ThreadPoolExecutor(max_workers=1, thread_name_prefix='artifact-writer')
        self.slots = threading.BoundedSemaphore(8)

    def submit(self, kind, path, function, *args):
        if not self.slots.acquire(blocking=False):
            self.report(kind, path, 'rejected', error='artifact queue full')
            return False
        try:
            # Freeze inputs before the mission continues using its observations.
            frozen = deepcopy(args)
            self.executor.submit(self.write, kind, path, function, frozen)
        except BaseException:
            self.slots.release()
            raise
        return True

    def write(self, kind, path, function, args):
        started = time.monotonic()
        try:
            result = function(path, *args)
        except Exception as exc:
            self.report(kind, path, 'failed', error=str(exc))
        else:
            self.report(kind, path, 'saved', elapsed_s=time.monotonic()-started, result=result)
        finally:
            self.slots.release()

    @staticmethod
    def report(kind, path, status, **details):
        line = json.dumps(dict(time=time.time(), event='artifact_write', kind=kind,
                               path=str(path), status=status, **details))
        print(line, file=sys.stderr, flush=True)
        try:
            with REPORT_LOCK:
                with (Path(path).parent/'artifact_writes.jsonl').open('a', encoding='utf-8') as log:
                    log.write(line+'\n')
        except OSError as exc:
            print('artifact write log failed: '+str(exc), file=sys.stderr, flush=True)

    def close(self):
        self.executor.shutdown(wait=True)
