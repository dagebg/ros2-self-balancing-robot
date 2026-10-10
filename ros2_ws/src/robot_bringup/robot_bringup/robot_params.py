"""Access to config/robot_params.yaml as installed with robot_description.

The YAML file is the single source of truth for robot constants (see docs: ADR-0004). Launch files
use these helpers instead of repeating values.
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml


def default_path() -> Path:
    """Location of robot_params.yaml in the robot_description share directory."""
    from ament_index_python.packages import get_package_share_directory

    return Path(get_package_share_directory("robot_description")) / "config" / "robot_params.yaml"


def load(path: Path | None = None) -> dict[str, Any]:
    """Loads the parameter file (the installed one by default)."""
    return yaml.safe_load((path or default_path()).read_text(encoding="utf-8"))


def value(params: dict[str, Any], group: str, name: str) -> Any:
    """Returns the value of one parameter, e.g. value(p, "geometry", "wheel_radius")."""
    return params[group][name]["value"]


def counts_per_wheel_rev(params: dict[str, Any]) -> float:
    """Encoder counts per wheel revolution with x4 quadrature decoding."""
    pulses = value(params, "drivetrain", "encoder_pulses_per_motor_rev")
    return 4.0 * float(pulses) * float(value(params, "drivetrain", "gear_ratio"))


def bridge_parameters(params: dict[str, Any]) -> dict[str, Any]:
    """Parameters robot_bridge needs from the robot description."""
    return {
        "wheel_radius": float(value(params, "geometry", "wheel_radius")),
        "track_width": float(value(params, "geometry", "track_width")),
        "counts_per_wheel_rev": counts_per_wheel_rev(params),
        "baud": int(value(params, "link", "uart_baud")),
    }


def placeholders(params: dict[str, Any]) -> list[str]:
    """Names of parameters that have not been measured yet, as "group.name"."""
    return [
        f"{group}.{name}"
        for group, entries in params.items()
        for name, entry in entries.items()
        if not entry["measured"]
    ]
