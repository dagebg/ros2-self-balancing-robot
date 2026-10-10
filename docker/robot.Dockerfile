# syntax=docker/dockerfile:1
# Robot image for the Raspberry Pi: ROS 2 Jazzy with the bridge, the model and the bringup.
# Only the packages the robot runs are built: no simulation, no GUI tools, no tests.
#
#   docker build -f docker/robot.Dockerfile -t robot .
#   docker run --rm --network host --device /dev/serial0 robot

ARG ROS_DISTRO=jazzy
FROM ros:${ROS_DISTRO}
ARG ROS_DISTRO

WORKDIR /ws
COPY config config
COPY shared shared
COPY ros2_ws/src/robot_interfaces ros2_ws/src/robot_interfaces
COPY ros2_ws/src/robot_bridge ros2_ws/src/robot_bridge
COPY ros2_ws/src/robot_description ros2_ws/src/robot_description
COPY ros2_ws/src/robot_bringup ros2_ws/src/robot_bringup

RUN apt-get update \
 && rosdep update --rosdistro "${ROS_DISTRO}" \
 && rosdep install --from-paths ros2_ws/src shared --ignore-src --rosdistro "${ROS_DISTRO}" -y \
      --skip-keys "rviz2 joint_state_publisher_gui" \
 && rm -rf /var/lib/apt/lists/*

RUN . "/opt/ros/${ROS_DISTRO}/setup.sh" \
 && cd ros2_ws \
 && colcon build --base-paths src ../shared/*/ --install-base /opt/robot \
      --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
 && rm -rf /ws

COPY docker/robot-entrypoint.sh /usr/local/bin/robot-entrypoint.sh
ENTRYPOINT ["/usr/local/bin/robot-entrypoint.sh"]
CMD ["ros2", "launch", "robot_bringup", "robot.launch.py"]
