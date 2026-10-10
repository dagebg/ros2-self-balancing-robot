"""robot_bringup.robot_params reads the installed config/robot_params.yaml correctly."""

import pytest
from robot_bringup import robot_params


def entry(value, measured=True):
    return {"value": value, "unit": "-", "measured": measured, "doc": "d"}


@pytest.fixture
def params():
    return {
        "geometry": {"wheel_radius": entry(0.05), "track_width": entry(0.2, measured=False)},
        "drivetrain": {
            "gear_ratio": entry(100.0),
            "encoder_pulses_per_motor_rev": entry(7),
        },
        "link": {"uart_baud": entry(460800)},
    }


def test_counts_per_wheel_rev_uses_x4_decoding(params):
    assert robot_params.counts_per_wheel_rev(params) == 4 * 7 * 100


def test_bridge_parameters(params):
    assert robot_params.bridge_parameters(params) == {
        "wheel_radius": 0.05,
        "track_width": 0.2,
        "counts_per_wheel_rev": 2800.0,
        "baud": 460800,
    }


def test_placeholders_lists_unmeasured_values(params):
    assert robot_params.placeholders(params) == ["geometry.track_width"]


def test_installed_file_is_the_repository_file():
    params = robot_params.load()
    bridge = robot_params.bridge_parameters(params)
    assert bridge["wheel_radius"] > 0
    assert bridge["counts_per_wheel_rev"] > 0
    assert bridge["baud"] == 460800
