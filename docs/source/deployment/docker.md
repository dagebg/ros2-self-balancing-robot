# Running on the Pi with Docker

CI publishes two images to the GitHub Container Registry for every push to `main` and every
release tag (see {doc}`../development/adr/0013-docker-deployment`):

| Image | Platforms | Contents |
|---|---|---|
| `ghcr.io/dagebg/ros2-self-balancing-robot/robot` | linux/arm64 | ROS 2 Jazzy, `robot_bridge`, `robot_description`, `robot_bringup`, rosbridge |
| `ghcr.io/dagebg/ros2-self-balancing-robot/web` | linux/amd64, linux/arm64 | the web app behind nginx |

Tags: `latest` (main), the version of each release (e.g. `0.1.0`), and `sha-<commit>`.

## One-time setup on the Pi

1. Set up the UART as described in {doc}`pi-setup`.
2. Install Docker Engine for Ubuntu (see Docker's documentation) and let your user run it:
   `sudo usermod -aG docker "$USER"`, then log in again.
3. Copy `docker/compose.yaml` from the repository to the Pi.

## Start, update, stop

```bash
docker compose -f compose.yaml up -d                 # start (latest main)
TAG=0.1.0 docker compose -f compose.yaml up -d       # a specific release
docker compose -f compose.yaml pull && docker compose -f compose.yaml up -d   # update
docker compose -f compose.yaml logs -f robot         # follow the robot's log
docker compose -f compose.yaml down                  # stop
```

With `restart: unless-stopped`, both containers come back after a reboot.

- The web app is served at `http://<pi>:8080` and connects to rosbridge on port 9090.
- The robot container uses host networking, so ROS 2 tools on other machines on the same network
  see its topics, and rosbridge listens on the Pi's own port 9090.
- `ROBOT_UART=/dev/ttyAMA0 docker compose … up -d` selects another serial device.

## Building the images yourself

```bash
docker buildx build -f docker/robot.Dockerfile -t robot --platform linux/arm64 --load .
docker buildx build -f docker/web.Dockerfile -t robot-web --load .
```

On a Raspberry Pi or an Apple-silicon Mac this builds natively; elsewhere `linux/arm64` runs
under emulation and is slow.

## Security

rosbridge has no authentication and the web app can arm the motors. Only expose ports 8080 and
9090 on a network you trust, such as the direct development link or the robot's own access point.
