"""Correlated HTTP timings without relying on clock synchronization between hosts."""

import json
import time
import urllib.error
import urllib.request
import uuid
from urllib.parse import urlsplit

from util.timing import timed_step


def request_json(url, headers, encode_payload=None, **fields):
    request_id = uuid.uuid4().hex
    headers = {**headers, "X-Request-ID": request_id}
    with timed_step("uav.http", request_id=request_id, path=urlsplit(url).path, **fields) as details:
        started = time.perf_counter()
        payload = encode_payload() if encode_payload is not None else None
        details["encode_ms"] = round((time.perf_counter() - started) * 1000, 3)
        details["request_bytes"] = len(payload) if payload is not None else 0
        request = urllib.request.Request(url, payload, headers)
        http = urllib.request.build_opener(urllib.request.ProxyHandler({}))
        started = time.perf_counter()
        try:
            with http.open(request, timeout=120) as response:
                details["wait_headers_ms"] = round((time.perf_counter() - started) * 1000, 3)
                details["http_status"] = response.status
                server_timing = response.headers.get("X-Server-Timing")
                body_started = time.perf_counter()
                body = response.read()
                details["response_body_read_ms"] = round((time.perf_counter() - body_started) * 1000, 3)
        except urllib.error.HTTPError as exc:
            details["http_status"] = exc.code
            raise
        finally:
            details["http_roundtrip_ms"] = round((time.perf_counter() - started) * 1000, 3)
        details["response_bytes"] = len(body)
        if server_timing:
            try:
                stages = json.loads(server_timing)
                server_ms = float(stages["server_ms"])
                details["server_stages_ms"] = stages
                # Includes network, TCP setup, scheduling, server accept queue and
                # response send; this is NOT a measured one-way return latency.
                details["transport_and_queue_ms"] = round(details["http_roundtrip_ms"] - server_ms, 3)
            except (ValueError, TypeError, KeyError):
                details["server_timing_invalid"] = True
        started = time.perf_counter()
        try:
            return json.loads(body)
        finally:
            details["json_decode_ms"] = round((time.perf_counter() - started) * 1000, 3)
