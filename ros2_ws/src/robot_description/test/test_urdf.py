"""The xacro model expands in both modes and matches config/robot_params.yaml."""

import math
import xml.etree.ElementTree as ET
from pathlib import Path

import pytest
import xacro
import yaml
from ament_index_python.packages import get_package_share_directory

SHARE = Path(get_package_share_directory("robot_description"))


def expand(sim: bool) -> ET.Element:
    document = xacro.process_file(
        str(SHARE / "urdf" / "robot.urdf.xacro"), mappings={"sim": str(sim).lower()}
    )
    return ET.fromstring(document.toxml())


def params() -> dict:
    return yaml.safe_load((SHARE / "config" / "robot_params.yaml").read_text())


def names(robot: ET.Element, tag: str) -> set[str]:
    return {element.get("name") for element in robot.findall(tag)}


def test_real_robot_has_footprint_and_pitch_joint():
    robot = expand(sim=False)
    assert {"base_footprint", "base_link", "left_wheel_link", "imu_link"} <= names(robot, "link")
    assert "body_pitch_joint" in names(robot, "joint")


def test_simulation_model_is_free_floating():
    robot = expand(sim=True)
    assert "base_footprint" not in names(robot, "link")
    assert "body_pitch_joint" not in names(robot, "joint")
    parents = {joint.find("parent").get("link") for joint in robot.findall("joint")}
    assert parents == {"base_link"}  # base_link is the root


@pytest.mark.parametrize("sim", [False, True])
def test_wheels_are_continuous_joints_about_y(sim):
    robot = expand(sim)
    for side in ("left", "right"):
        joint = robot.find(f"joint[@name='{side}_wheel_joint']")
        assert joint.get("type") == "continuous"
        assert joint.find("axis").get("xyz") == "0 1 0"


def test_geometry_and_masses_come_from_robot_params():
    p = params()["geometry"]
    robot = expand(sim=False)
    left = robot.find("joint[@name='left_wheel_joint']/origin").get("xyz").split()
    assert math.isclose(float(left[1]), p["track_width"]["value"] / 2)
    radius = robot.find("link[@name='left_wheel_link']/visual/geometry/cylinder").get("radius")
    assert math.isclose(float(radius), p["wheel_radius"]["value"])
    mass = robot.find("link[@name='base_link']/inertial/mass").get("value")
    assert math.isclose(float(mass), p["body_mass"]["value"])
    pitch_origin = robot.find("joint[@name='body_pitch_joint']/origin").get("xyz").split()
    assert math.isclose(float(pitch_origin[2]), p["wheel_radius"]["value"])


def test_simulation_model_has_gazebo_sensors_and_plugins():
    robot = expand(sim=True)
    sensor = robot.find("gazebo[@reference='base_link']/sensor[@type='imu']")
    assert sensor is not None
    plugins = [p.get("filename") for p in robot.findall("gazebo/plugin")]
    assert plugins.count("gz-sim-apply-joint-force-system") == 2
    assert "gz-sim-joint-state-publisher-system" in plugins


def test_real_robot_model_has_no_gazebo_tags():
    assert expand(sim=False).find("gazebo") is None
