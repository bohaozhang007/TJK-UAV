"""Write per-camera-job timing traces without disk I/O in the control loop."""
import copy
import json
import logging
from pathlib import Path
import queue
import threading
import time


class AutofocusTrace:
    def __init__(self, path, task_id):
        self.path = Path(path)
        self.task_id = task_id
        self.queue = queue.SimpleQueue()
        self.worker = threading.Thread(target=self._write, name='autofocus-trace', daemon=True)
        self.worker.start()

    def emit(self, event, **fields):
        self.queue.put(dict(time_s=time.time(), monotonic_s=time.monotonic(),
                            task_id=self.task_id, event=event, **copy.deepcopy(fields)))

    def _write(self):
        try:
            self.path.parent.mkdir(parents=True, exist_ok=True)
            with self.path.open('a', encoding='utf-8') as file:
                while True:
                    record = self.queue.get()
                    if record is None:
                        break
                    file.write(json.dumps(record, ensure_ascii=False)+'\n')
                    file.flush()
        except Exception:
            logging.exception('Autofocus trace write failed: %s', self.path)

    def close(self):
        self.queue.put(None)
        self.worker.join()
