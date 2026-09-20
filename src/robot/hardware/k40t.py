"""K40T UDP zoom and relative gimbal control using measured joint angles."""

from __future__ import annotations

import math
import secrets
import socket
import struct
import threading
import time


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for byte in data:
        tmp = byte ^ (crc & 0xFF)
        tmp = (tmp ^ (tmp << 4)) & 0xFF
        crc = ((crc >> 8) ^ (tmp << 8) ^ (tmp << 3) ^ (tmp >> 4)) & 0xFFFF
    return crc


def encode_packet(message_id: int, payload: bytes, sequence: int) -> bytes:
    body = bytes((len(payload), 4, 1, sequence, 1, 1))
    body += message_id.to_bytes(3, "little") + payload
    return b"\xfd" + body + struct.pack("<H", crc16(body))


def decode_packet(packet: bytes):
    if len(packet) < 12 or packet[0] != 0xFD or len(packet) != packet[1] + 12:
        return None
    if packet[2:4] != b"\x01\x01" or packet[5:7] != b"\x04\x01":
        return None
    if crc16(packet[1:-2]) != int.from_bytes(packet[-2:], "little"):
        return None
    return int.from_bytes(packet[7:10], "little"), packet[4], packet[10:-2]


class K40TClient:
    def __init__(self, host: str, port: int, timeout_s: float):
        if not host or not 1 <= port <= 65535 or not math.isfinite(timeout_s) or timeout_s <= 0:
            raise ValueError("Invalid K40T address or timeout")
        self.address = (host, port)
        self.timeout_s = timeout_s
        self._lock = threading.RLock()
        self._sequence = secrets.randbelow(256)
        self._last_gimbal = None

    def get_gimbal(self) -> dict:
        return self._request(0x000200, b"\x01\x00", None, kind="gimbal")

    def gimbal_yaw(self, delta_deg: float) -> dict:
        """Relative yaw in degrees, positive right, negative left."""
        return self._move_gimbal("yaw_deg", delta_deg)

    def gimbal_pitch(self, delta_deg: float) -> dict:
        """Relative pitch in degrees, positive up, negative down."""
        return self._move_gimbal("pitch_deg", delta_deg)

    def _move_gimbal(self, axis: str, delta_deg: float) -> dict:
        if isinstance(delta_deg, bool):
            raise ValueError("Gimbal angle must be a finite number of degrees")
        delta = float(delta_deg)
        if not math.isfinite(delta) or abs(delta) > 360:
            raise ValueError("Gimbal angle must be finite and within +/-360 degrees")
        if not math.isclose(delta * 100, round(delta * 100), abs_tol=1e-8, rel_tol=0):
            raise ValueError("Gimbal angle supports at most two decimal places")
        with self._lock:
            # Always refresh; never integrate requested deltas into a guessed pose.
            start = self.get_gimbal()
            target = {key: round(start[key], 2) for key in ("yaw_deg", "pitch_deg")}
            target[axis] = round(target[axis] + delta, 2)
            for key, low, high in (("yaw_deg", -180, 180), ("pitch_deg", -90, 30)):
                if not low <= target[key] <= high:
                    raise ValueError(
                        f"Gimbal target {key}={target[key]} outside [{low}, {high}]; "
                        f"current={start[key]}, delta={delta}. No movement sent."
                    )
            if delta == 0:
                return dict(start, message="zero delta; no movement sent")
            pitch, yaw = target["pitch_deg"], target["yaw_deg"]
            # 0x12 uses absolute joint targets (verified on this K40T).
            # Explicitly hold the other axis: firmware moved pitch with the
            # documented direction=2/no-motion value during a yaw-only test.
            payload = struct.pack(
                "<BHBHB", 0 if pitch >= 0 else 1, round(abs(pitch) * 100),
                1 if yaw >= 0 else 0, round(abs(yaw) * 100), 0,
            )
            result = self._request(0x12, payload, target, kind="gimbal", mount=start["mount"])
            return dict(result, delta_deg=delta, axis=axis, target=target, start=start)

    def get_zoom(self) -> dict:
        # Read all visible-light settings to establish the UDP return endpoint;
        # the camera then sends its periodic visible-light status (0x020005).
        return self._request(0x000200, b"\x01\x00", None)

    def set_zoom(self, ratio: float) -> dict:
        if isinstance(ratio, bool):
            raise ValueError("zoom requires a multiplier from 1 to 160")
        ratio = float(ratio)
        if not math.isfinite(ratio) or not 1 <= ratio <= 160:
            raise ValueError("zoom requires a multiplier from 1 to 160")
        tenths = round(ratio * 10)
        if not math.isclose(ratio * 10, tenths, abs_tol=1e-8, rel_tol=0):
            raise ValueError("zoom supports at most one decimal place, e.g. zoom 1.5")
        return self._request(0x000304, struct.pack("<BH", 0, tenths), tenths)

    def _request(self, message_id: int, payload: bytes, target, *, kind="zoom", mount=None) -> dict:
        with self._lock, socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
            sock.connect(self.address)
            self._sequence = (self._sequence + 1) % 256
            sequence = self._sequence
            sock.send(encode_packet(message_id, payload, sequence))
            deadline = time.monotonic() + self.timeout_s
            acknowledged = False
            last_status = None
            stable_samples = 0
            while time.monotonic() < deadline:
                sock.settimeout(max(0.001, min(0.5, deadline - time.monotonic())))
                try:
                    packet = decode_packet(sock.recv(4096))
                except socket.timeout:
                    # UDP can lose a query/ACK. Only retry the read-only query;
                    # never resend a zoom command whose outcome is unknown.
                    if target is None and not acknowledged and time.monotonic() < deadline:
                        self._sequence = (self._sequence + 1) % 256
                        sequence = self._sequence
                        sock.send(encode_packet(message_id, payload, sequence))
                    continue
                if packet is None:
                    continue
                msg, seq, data = packet
                if msg == (message_id | 0x010000) and seq == sequence and len(data) >= 2:
                    if data[:2] != b"\x00\x00":
                        raise RuntimeError(f"K40T rejected command: ACK={data[:2].hex()}")
                    acknowledged = True
                elif kind == "gimbal" and acknowledged and msg in (1, 0x020001) and len(data) >= 7:
                    if data[0] & 0x0F:
                        raise RuntimeError("K40T reports a gimbal connection fault")
                    if data[4] not in (0, 1):
                        raise RuntimeError(f"Unknown K40T mounting orientation: {data[4]}")
                    if mount is not None and mount != data[4]:
                        raise RuntimeError("K40T mounting orientation changed during command")
                    mount = data[4]
                elif kind == "gimbal" and acknowledged and mount is not None and msg in (2, 0x020002) and len(data) >= 20:
                    yaw_raw, roll_raw, pitch_raw = struct.unpack_from("<hhh", data)
                    if abs(yaw_raw) > 18000 or abs(pitch_raw) > 18000:
                        raise RuntimeError("K40T reported invalid joint angles")
                    pitch = pitch_raw / 100.0
                    if mount == 1:
                        pitch = 180 - pitch if pitch > 0 else -180 - pitch
                    last_status = {
                        "ok": True, "yaw_deg": yaw_raw / 100.0,
                        "pitch_deg": round(pitch, 2), "mount": mount,
                        "raw_joint_deg": {"yaw": yaw_raw / 100.0, "pitch": pitch_raw / 100.0, "roll": roll_raw / 100.0},
                        "reference": "gimbal_zero", "received_at": time.time(),
                    }
                    self._last_gimbal = dict(last_status)
                    if target is None:
                        return last_status
                    # Compare joint angles directly, without wrapping across
                    # +/-180: the physical travel limit must not be bypassed.
                    reached = all(abs(last_status[key] - target[key]) <= 0.5 for key in ("yaw_deg", "pitch_deg"))
                    stable_samples = stable_samples + 1 if reached else 0
                    if stable_samples >= 3:
                        return last_status
                elif kind == "zoom" and msg in (0x000005, 0x020005) and len(data) >= 15 and acknowledged:
                    status, focal, zoom = struct.unpack_from("<BHH", data)
                    if status not in (0, 1) or not 10 <= zoom <= 1600:
                        continue
                    last_status = {
                        "ok": True,
                        "zoom": zoom / 10.0,
                        "zooming": status == 1,
                        "focal_length_mm": focal / 100.0,
                    }
                    if target is None or (zoom == target and status == 0):
                        if target is not None:
                            last_status["requested_zoom"] = target / 10.0
                        return last_status
            raise RuntimeError(
                f"K40T {kind} confirmation timed out (acknowledged={acknowledged}, "
                f"last_status={last_status}); actual {kind} is unconfirmed, use get_{kind}"
            )
