# Releasing

Versions follow [Semantic Versioning](https://semver.org/). A release is a Git tag `vX.Y.Z` on
`main`; everything else is automated.

## Steps

1. Move the entries under **Unreleased** in `CHANGELOG.md` to a new section `## [X.Y.Z] - date`
   and bump `release` in `docs/source/conf.py`; merge that through a pull request.
2. Preview the generated release notes: `.venv/bin/git-cliff --unreleased`.
3. Tag and push:

   ```bash
   git switch main && git pull
   git tag -a vX.Y.Z -m "vX.Y.Z"
   git push origin vX.Y.Z
   ```

## What the tag triggers

| Workflow | Result |
|---|---|
| `release.yml` | GitHub Release with release notes from the Conventional Commits since the previous tag, and these files: |
| | `robot-firmware-vX.Y.Z.bin`: complete flash image (bootloader, partitions, app), flashed at `0x0` |
| | `robot-bringup-vX.Y.Z.bin`: the hardware bring-up tool, same format |
| | `robot-firmware-vX.Y.Z.elf`: symbols for decoding crash backtraces |
| | `robot-webapp-vX.Y.Z.zip`: the built web app |
| `docker.yml` | `robot` and `web` images tagged `X.Y.Z` in GHCR |
| `docs.yml` | documentation is rebuilt from `main` |

## Flashing a release image

No toolchain needed, only `esptool` (`pip install esptool`):

```bash
esptool.py --chip esp32 --port /dev/ttyUSB0 write_flash 0x0 robot-firmware-vX.Y.Z.bin
```

The merged image is produced by `tools/merge_firmware.sh`, which can also be run locally after
`pio run -e esp32`.
