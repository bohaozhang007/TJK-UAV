"""Wall-clock stage timings. Durations use a local monotonic clock, in milliseconds."""

import json
import threading
import time
from contextlib import contextmanager
from datetime import datetime
from functools import wraps
from inspect import signature


_print_lock = threading.RLock()


def log_timing(event, **fields):
    record = {
        "time": datetime.now().astimezone().isoformat(timespec="milliseconds"),
        "event": event,
        **fields,
    }
    with _print_lock:
        print("[TIMING] " + json.dumps(record, ensure_ascii=True, allow_nan=False), flush=True)


@contextmanager
def timed_step(event, **fields):
    """Yield mutable details and log both successful and failed operations."""
    started = time.perf_counter()
    details = dict(fields)
    try:
        yield details
    except BaseException as exc:
        details.update(status="error", error_type=type(exc).__name__, error=str(exc))
        raise
    finally:
        details.setdefault("status", "ok")
        details["duration_ms"] = round((time.perf_counter() - started) * 1000, 3)
        log_timing(event, **details)


def timed(event):
    """Time a function without logging images or other large/sensitive arguments."""
    def decorate(function):
        parameters = signature(function)

        @wraps(function)
        def wrapped(*args, **kwargs):
            bound = parameters.bind_partial(*args, **kwargs).arguments
            fields = {key: bound[key] for key in ("target_id", "frame_id", "pitch_deg", "yaw_deg", "ratio", "topic") if key in bound}
            target = bound.get("target")
            if isinstance(target, dict):
                fields["target_id"] = target.get("target_id")
            with timed_step(event, **fields) as details:
                result = function(*args, **kwargs)
                if isinstance(result, bool):
                    details["result"] = result
                return result
        return wrapped
    return decorate
