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

## Everyday commands

| Task | Command |
|---|---|
| Lint and format everything | `.venv/bin/pre-commit run --all-files` |
| Python tool tests | `.venv/bin/pytest` |
| Regenerate `robot_config.hpp` | `.venv/bin/python tools/gen_config.py` |
| Firmware unit tests (host) | `cd firmware/esp32/robot-firmware && pio test -e native` |
| Firmware build | `cd firmware/esp32/robot-firmware && pio run -e esp32` |
| Firmware static analysis | `cd firmware/esp32/robot-firmware && pio check -e esp32` |
| Build the docs | `make -C docs html SPHINXBUILD=../.venv/bin/sphinx-build SPHINXOPTS="-W --keep-going"` |

(`pio` is `.venv/bin/pio` unless the venv is activated.)

## Troubleshooting (macOS)

Host-side C++ builds (`pio test -e native`, CMake) link with Apple's toolchain. Two local setups
are known to break that link step with `tapi error: unknown architecture arm64e.x1`:

- **Conda on `PATH`:** conda ships its own, older `ld`. Deactivate conda (or drop its `bin` from
  `PATH`) for the build.
- **An SDK newer than the Command Line Tools:** if `xcrun --show-sdk-path` points at an SDK
  newer than the installed linker supports, point the build at a matching one, e.g.
  `export SDKROOT=/Library/Developer/CommandLineTools/SDKs/MacOSX26.sdk`, or update the
  Command Line Tools.
