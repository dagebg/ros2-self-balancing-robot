# ROS 2 Jazzy development image: the official ros:jazzy image plus every dependency declared in
# the repository's package.xml files. Built by tools/ros-docker.sh from a context that contains
# only those package.xml files, so source changes do not invalidate the dependency layer.
FROM ros:jazzy

COPY . /tmp/packages
RUN apt-get update \
 && rosdep update --rosdistro jazzy \
 && rosdep install --from-paths /tmp/packages --ignore-src --rosdistro jazzy -y \
 && rm -rf /var/lib/apt/lists/* /tmp/packages

ENV RCUTILS_COLORIZED_OUTPUT=0
WORKDIR /ws
