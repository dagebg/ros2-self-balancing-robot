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
| Firmware static analysis | `cd firmware/esp32/robot-firmware && ../../../.venv/bin/pio check` |
| Build the docs | `make -C docs html SPHINXBUILD=../.venv/bin/sphinx-build SPHINXOPTS="-W --keep-going"` |

Commands for firmware builds, unit tests and ROS 2 are added to this page as those parts land.
