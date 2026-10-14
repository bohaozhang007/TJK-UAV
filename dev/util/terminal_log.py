import sys
import threading
from contextlib import contextmanager
from datetime import datetime
from pathlib import Path


LOG_DIR = Path(__file__).resolve().parents[2] / "logs"
TIMESTAMP_FORMAT = "%Y%m%d_%H%M%S_%f"


class TeeStream:
    def __init__(
        self,
        terminal,
        logfile,
        lock,
    ):
        self.terminal = terminal
        self.logfile = logfile
        self.lock = lock

    def write(self, text):
        with self.lock:
            self.terminal.write(text)
            self.logfile.write(text)
            # Flush each write because ROS shutdown can call os._exit().
            self.terminal.flush()
            self.logfile.flush()
        return len(text)

    def flush(self):
        with self.lock:
            self.terminal.flush()
            self.logfile.flush()

    def __getattr__(self, name):
        return getattr(self.terminal, name)


@contextmanager
def save_terminal_output():
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.now().strftime(TIMESTAMP_FORMAT)
    run_dir = LOG_DIR / timestamp
    run_dir.mkdir()
    path = run_dir / "log.txt"
    stdout, stderr = sys.stdout, sys.stderr
    lock = threading.RLock()
    with path.open("x", encoding="utf-8") as logfile:
        sys.stdout = TeeStream(
            stdout,
            logfile,
            lock,
        )
        sys.stderr = TeeStream(
            stderr,
            logfile,
            lock,
        )
        try:
            print(f"Terminal log: {path}", flush=True)
            yield path
        finally:
            sys.stdout, sys.stderr = stdout, stderr

