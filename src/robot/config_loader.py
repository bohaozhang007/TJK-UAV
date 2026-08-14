from __future__ import annotations

import math
from pathlib import Path
from typing import Any, Mapping

import yaml


_REPOSITORY_ROOT = Path(__file__).resolve().parents[2]
_DEFAULT_CONFIG_PATHS = {
    "common": Path(__file__).resolve().parent / "config" / "common.yaml",
    "tello": Path(__file__).resolve().parent / "config" / "tello.yaml",
    "ue": Path(__file__).resolve().parent / "config" / "ue.yaml",
    "owl": Path(__file__).resolve().parent / "config" / "owl.yaml",
    "i7": _REPOSITORY_ROOT / "ros" / "i7_nav" / "config" / "i7_nav.yaml",
}


def load_robot_config(
    name: str,
    config_path: str | Path | None = None,
) -> dict[str, Any]:
    if config_path is None:
        try:
            path = _DEFAULT_CONFIG_PATHS[name]
        except KeyError as exc:
            raise ValueError(f"Unknown Robot config name: {name!r}") from exc
    else:
        path = Path(config_path).expanduser().resolve()
    try:
        with path.open("r", encoding="utf-8") as config_file:
            config = yaml.safe_load(config_file)
    except (OSError, yaml.YAMLError) as exc:
        raise RuntimeError(f"Failed to load Robot config: {path}") from exc
    if not isinstance(config, dict):
        raise ValueError(f"Robot config must be a mapping: {path}")
    return config


def required_section(config: Mapping[str, Any], key: str) -> dict[str, Any]:
    value = config.get(key)
    if not isinstance(value, dict):
        raise ValueError(f"Missing Robot config section: {key}")
    return value


def required_number(
    config: Mapping[str, Any],
    key: str,
    *,
    integer: bool = False,
    minimum: float | None = None,
    maximum: float | None = None,
) -> int | float:
    value = config.get(key)
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"Robot config value {key} must be numeric, got {value!r}")
    result = int(value) if integer else float(value)
    if integer and float(value) != result:
        raise ValueError(f"Robot config value {key} must be an integer, got {value!r}")
    if not math.isfinite(float(result)):
        raise ValueError(f"Robot config value {key} must be finite")
    if minimum is not None and result < minimum:
        raise ValueError(f"Robot config value {key} must be >= {minimum}")
    if maximum is not None and result > maximum:
        raise ValueError(f"Robot config value {key} must be <= {maximum}")
    return result


def required_string(config: Mapping[str, Any], key: str) -> str:
    value = config.get(key)
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"Robot config value {key} must be a non-empty string")
    return value


def required_bool(config: Mapping[str, Any], key: str) -> bool:
    value = config.get(key)
    if not isinstance(value, bool):
        raise ValueError(f"Robot config value {key} must be boolean")
    return value


__all__ = [
    "load_robot_config",
    "required_bool",
    "required_number",
    "required_section",
    "required_string",
]
