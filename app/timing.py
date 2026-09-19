"""Request-local timing records, including failed stages."""
from contextlib import contextmanager
import time


@contextmanager
def measure(timings, name):
    started = time.monotonic()
    record = dict(status='ok')
    try:
        yield
    except BaseException as exc:
        record.update(status='error', error=str(exc) or type(exc).__name__)
        raise
    finally:
        record['elapsed_s'] = time.monotonic() - started
        if timings is not None:
            timings[name] = record
