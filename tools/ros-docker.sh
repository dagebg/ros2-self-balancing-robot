#!/usr/bin/env bash
# Runs a command in the ROS 2 Jazzy development container with the repository mounted at /ws.
#
#   tools/ros-docker.sh colcon-build      build all ROS 2 and shared packages
#   tools/ros-docker.sh colcon-test       build, then run all tests
#   tools/ros-docker.sh <command...>      any command, e.g. "ros2 topic list"
#   ROS_DOCKER_ARGS="-p 9090:9090" tools/ros-docker.sh ...   pass extra docker arguments
#
# The image is rebuilt automatically when any package.xml changes (its tag is their hash).
set -euo pipefail

ROOT="$(git rev-parse --show-toplevel)"
cd "$ROOT"

PACKAGES=()
while IFS= read -r package; do  # no mapfile: macOS ships bash 3.2
  PACKAGES+=("$package")
done < <(git ls-files --cached --others --exclude-standard '*package.xml' \
  | grep -E '^(ros2_ws/src|shared)/' | sort)
HASH="$(cat "${PACKAGES[@]}" docker/dev.Dockerfile | shasum | cut -c1-12)"
IMAGE="robot-ros2-dev:${HASH}"

if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  CONTEXT="$(mktemp -d)"
  trap 'rm -rf "$CONTEXT"' EXIT
  for package in "${PACKAGES[@]}"; do
    mkdir -p "$CONTEXT/$(dirname "$package")"
    cp "$package" "$CONTEXT/$package"
  done
  docker build -t "$IMAGE" -f docker/dev.Dockerfile "$CONTEXT"
fi

# The shared libraries are listed one by one: shared/CMakeLists.txt is the standalone test
# aggregator and would otherwise be picked up as a package that hides them.
COLCON_ARGS=(--base-paths src "../shared/*/" --event-handlers console_cohesion+)
case "${1:-}" in
  colcon-build)
    CMD="cd ros2_ws && colcon build ${COLCON_ARGS[*]} --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo"
    ;;
  colcon-test)
    CMD="cd ros2_ws && colcon build ${COLCON_ARGS[*]} --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo \
      && colcon test ${COLCON_ARGS[*]} --return-code-on-test-failure \
      ; status=\$?; colcon test-result --verbose; exit \$status"
    ;;
  "")
    CMD="bash"
    ;;
  *)
    CMD="$*"
    ;;
esac

TTY_FLAGS=()
if [ -t 0 ] && [ -t 1 ]; then
  TTY_FLAGS=(-it)
fi

# Extra docker arguments, e.g. ROS_DOCKER_ARGS="-p 9090:9090" to reach rosbridge from the host.
read -r -a EXTRA_ARGS <<< "${ROS_DOCKER_ARGS:-}"

exec docker run --rm "${TTY_FLAGS[@]}" ${EXTRA_ARGS[@]+"${EXTRA_ARGS[@]}"} -v "$ROOT:/ws" -w /ws "$IMAGE" \
  bash -c "source /opt/ros/jazzy/setup.bash && $CMD"
