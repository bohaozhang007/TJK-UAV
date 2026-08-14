from __future__ import annotations

import threading
from pathlib import Path
from typing import Any, Protocol

from .config_loader import load_robot_config, required_number, required_section

class VelocityController(Protocol):
    def velocity(self, x: int, y: int, z: int, yaw: int) -> dict[str, Any]: ...

    def land(self) -> dict[str, Any]: ...

    def health(self) -> dict[str, Any]: ...


class KeyboardOp:
    """Foreground keyboard operation for a velocity-capable controller."""

    _MOTION_KEYS = frozenset({"t", "g", "f", "h", "i", "k", "j", "l"})

    def __init__(
        self,
        controller: VelocityController,
        *,
        config: dict[str, Any] | None = None,
        config_path: str | Path | None = None,
    ) -> None:
        keyboard_config = required_section(
            config or load_robot_config("common", config_path),
            "keyboard",
        )
        min_command = required_number(
            keyboard_config,
            "min_command_percent",
            integer=True,
            minimum=0.0,
        )
        max_command = required_number(
            keyboard_config,
            "max_command_percent",
            integer=True,
            minimum=float(min_command),
        )
        min_frequency = required_number(
            keyboard_config, "min_frequency_hz", minimum=1e-6
        )
        max_frequency = required_number(
            keyboard_config,
            "max_frequency_hz",
            minimum=float(min_frequency),
        )
        self.controller = controller
        self.speed = max(
            int(min_command),
            min(
                int(max_command),
                required_number(
                    keyboard_config,
                    "speed_percent",
                    integer=True,
                ),
            ),
        )
        self.yaw_speed = max(
            int(min_command),
            min(
                int(max_command),
                required_number(
                    keyboard_config,
                    "yaw_speed_percent",
                    integer=True,
                ),
            ),
        )
        self.frequency_hz = max(
            float(min_frequency),
            min(
                float(max_frequency),
                required_number(keyboard_config, "frequency_hz"),
            ),
        )
        self._pressed: set[str] = set()
        self._state_lock = threading.RLock()
        self._stop_event = threading.Event()
        self._land_requested = threading.Event()
        self._active = False

    def is_active(self) -> bool:
        with self._state_lock:
            return self._active

    def state(self) -> dict[str, Any]:
        with self._state_lock:
            return {
                "active": self._active,
                "pressed": sorted(self._pressed),
                "speed": self.speed,
                "yaw_speed": self.yaw_speed,
                "frequency_hz": self.frequency_hz,
            }

    def stop(self) -> None:
        self._stop_event.set()

    def run_foreground(self) -> dict[str, Any]:
        try:
            from pynput import keyboard as pynput_keyboard
        except ImportError as exc:
            raise RuntimeError(
                "keyboard control requires the optional pynput package"
            ) from exc

        health = self.controller.health()
        if not health.get("initialized"):
            return {"ok": False, "error": "drone is not initialized, call init first"}
        if not health.get("airborne"):
            return {"ok": False, "error": "drone is not airborne, call takeoff first"}
        if self.is_active():
            return {"ok": False, "error": "keyboard operation is already active"}

        with self._state_lock:
            self._pressed.clear()
            self._active = True
        self._stop_event.clear()
        self._land_requested.clear()
        listener = None
        final_error = ""
        land_result: dict[str, Any] | None = None

        def key_char(key: Any) -> str:
            char = getattr(key, "char", "")
            return char.lower() if isinstance(char, str) else ""

        def on_press(key: Any):
            if key == pynput_keyboard.Key.esc:
                self._stop_event.set()
                return False
            char = key_char(key)
            if char == "b":
                self._land_requested.set()
                self._stop_event.set()
                return False
            with self._state_lock:
                if key == pynput_keyboard.Key.space:
                    self._pressed.add("space")
                else:
                    if char in self._MOTION_KEYS:
                        self._pressed.add(char)
            return None

        def on_release(key: Any) -> None:
            if key == pynput_keyboard.Key.space:
                with self._state_lock:
                    self._pressed.discard("space")
                return
            char = key_char(key)
            if char:
                with self._state_lock:
                    self._pressed.discard(char)

        try:
            listener = pynput_keyboard.Listener(on_press=on_press, on_release=on_release)
            listener.start()
            interval = 1.0 / self.frequency_hz

            while not self._stop_event.is_set():
                x, y, z, yaw = self._current_command()
                self.controller.velocity(x, y, z, yaw)
                self._stop_event.wait(interval)
        except (KeyboardInterrupt, EOFError):
            self._stop_event.set()
        except Exception as exc:
            final_error = str(exc)
        finally:
            if listener is not None:
                try:
                    listener.stop()
                except Exception:
                    pass
            try:
                self.controller.velocity(0, 0, 0, 0)
            except Exception as exc:
                final_error = final_error or str(exc)
            if self._land_requested.is_set():
                try:
                    land_result = self.controller.land()
                    if not land_result.get("ok", False):
                        final_error = final_error or str(
                            land_result.get("error") or "landing failed"
                        )
                except Exception as exc:
                    final_error = final_error or str(exc)
            with self._state_lock:
                self._pressed.clear()
                self._active = False
            self._stop_event.clear()
            self._land_requested.clear()

        if final_error:
            response = {"ok": False, "error": final_error, **self.state()}
            if land_result is not None:
                response["land"] = land_result
            return response
        if land_result is not None:
            return {
                "ok": True,
                "message": "keyboard operation landed",
                "land": land_result,
                **self.state(),
            }
        return {"ok": True, "message": "keyboard operation stopped", **self.state()}

    def _current_command(self) -> tuple[int, int, int, int]:
        with self._state_lock:
            pressed = set(self._pressed)

        if "space" in pressed:
            return 0, 0, 0, 0

        x = self.speed * (int("t" in pressed) - int("g" in pressed))
        y = self.speed * (int("h" in pressed) - int("f" in pressed))
        z = self.speed * (int("i" in pressed) - int("k" in pressed))
        yaw = self.yaw_speed * (int("l" in pressed) - int("j" in pressed))
        return x, y, z, yaw
