"""Client for the Unreal Engine simulator Robot Server."""

from .base import BaseClient, JsonObject


class UEClient(BaseClient):
    """UE client with optional simultaneous XYZ/yaw waypoint control.

    Set ``USE_HYBRID_CONTROL`` to ``False`` (or pass
    ``use_hybrid_control=False``) to fall back to the inherited sequential
    ``/move_relative_xyz`` followed by ``/rotate`` implementation.
    """

    USE_HYBRID_CONTROL = True

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 8765,
        timeout_s: float = 180.0,
        *,
        use_hybrid_control: bool | None = None,
    ) -> None:
        super().__init__(host, port, timeout_s)
        self.use_hybrid_control = (
            self.USE_HYBRID_CONTROL
            if use_hybrid_control is None
            else bool(use_hybrid_control)
        )

    def move_relative(
        self,
        dx: float = 0.0,
        dy: float = 0.0,
        dz: float = 0.0,
        dyaw: float = 0.0,
    ) -> JsonObject:
        if not self.use_hybrid_control:
            return super().move_relative(dx=dx, dy=dy, dz=dz, dyaw=dyaw)

        x, y, z = (int(round(value)) for value in (dx, dy, dz))
        yaw = int(round(dyaw))
        if not any((x, y, z, yaw)):
            return {
                "ok": True,
                "message": "move_relative_xyz_yaw skipped",
                "command": {"x": x, "y": y, "z": z, "yaw": yaw},
            }

        return self._require_ok(
            self._request_json(
                "POST",
                "/move_relative_xyz_yaw",
                {"x": x, "y": y, "z": z, "yaw": yaw},
            ),
            "move_relative_xyz_yaw",
        )
