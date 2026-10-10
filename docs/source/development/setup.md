# Development environment

Everything needed to build, test and document the project runs on macOS or Linux. Python tools are
pinned in `requirements-dev.txt` and `docs/requirements.txt`, so local runs match CI exactly.

## One-time setup

```bash
# Python tools (PlatformIO, ruff, clang-format, pre-commit, cmake, gcovr)
python3 -m venv .venv
.venv/bin/pip install -r requirements-dev.txt -r docs/requirements.txt

# Git hooks: formatting and linting run on every commit
.venv/bin/pre-commit install
```

System packages:

| Tool | macOS (Homebrew) | Ubuntu / Fedora | Used for |
|---|---|---|---|
| Doxygen | `brew install doxygen` | `apt install doxygen` / `dnf install doxygen` | API reference |
| Docker | `brew install colima docker` then `colima start` | Docker Engine | ROS 2 builds and tests in `ros:jazzy` |
| Node.js | `brew install node` | `apt install nodejs npm` | Web app |

On Apple Silicon, `colima` runs an arm64 Linux VM, the same CPU architecture as the Raspberry Pi,
so ROS 2 containers behave like they do on the robot.

`tools/ros-docker.sh` builds a development image from `ros:jazzy` plus every dependency declared in
the repository's `package.xml` files (via `rosdep`), mounts the repository at `/ws` and runs the
given command. The image is rebuilt automatically when a `package.xml` changes.

## Everyday commands

| Task | Command |
|---|---|
| Lint and format everything | `.venv/bin/pre-commit run --all-files` |
| Python tool tests | `.venv/bin/pytest` |
| Regenerate `robot_config.hpp` | `.venv/bin/python tools/gen_config.py` |
| Shared libraries: build + test | `cmake -S shared -B build/shared -G Ninja && cmake --build build/shared && ctest --test-dir build/shared` |
| … with sanitizers | add `-DROBOT_SANITIZERS=ON` (separate build directory) |
| … with coverage | add `-DROBOT_COVERAGE=ON`, then `gcovr -r . --filter 'shared/.*/(src\|include)/' build/shared` |
| clang-tidy (shared) | `clang-tidy -p build/shared shared/*/src/*.cpp` |
| Firmware unit tests (host) | `cd firmware/esp32/robot-firmware && pio test -e native` |
| Firmware build | `cd firmware/esp32/robot-firmware && pio run -e esp32` |
| Firmware static analysis | `cd firmware/esp32/robot-firmware && pio check -e esp32` |
| ROS 2: build all packages (Docker) | `tools/ros-docker.sh colcon-build` |
| ROS 2: build and run all tests | `tools/ros-docker.sh colcon-test` |
| ROS 2: shell in the container | `tools/ros-docker.sh` |
| Simulation (headless) | `tools/ros-docker.sh "cd ros2_ws && source install/setup.bash && ros2 launch robot_sim sim.launch.py"` |
| Build the docs | `make -C docs html SPHINXBUILD=../.venv/bin/sphinx-build SPHINXOPTS="-W --keep-going"` |

(`pio`, `cmake`, `ctest`, `gcovr` and `clang-tidy` come from `.venv/bin` unless the venv is
activated. On macOS, gcovr needs `--gcov-executable "xcrun llvm-cov gcov"`.)

## Troubleshooting (macOS)

Host-side C++ builds (`pio test -e native`, CMake) link with Apple's toolchain. Two local setups
are known to break that link step with `tapi error: unknown architecture arm64e.x1`:

- **Conda on `PATH`:** conda ships its own, older `ld`. Deactivate conda (or drop its `bin` from
  `PATH`) for the build.
- **An SDK newer than the Command Line Tools:** if `xcrun --show-sdk-path` points at an SDK
  newer than the installed linker supports, point the build at a matching one, e.g.
  `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.sdk`, or update the
  Command Line Tools.
