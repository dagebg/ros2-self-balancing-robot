# Raspberry Pi setup

The Pi runs Ubuntu Server 24.04 (64-bit) with ROS 2 Jazzy. This page covers the parts specific to
this robot; for the network setup during development see {doc}`networking`.

## UART for the ESP32 link

The ESP32 is wired to the Pi's GPIO 14 (TXD) and GPIO 15 (RXD). On a Pi 4 these pins are served by
the *mini UART* by default, whose baud rate depends on the CPU clock. Swap them to the full PL011
UART by moving Bluetooth off it, and stop Linux from using the port as a console.

1. In `/boot/firmware/config.txt`, add:

   ```ini
   enable_uart=1
   dtoverlay=disable-bt
   ```

2. In `/boot/firmware/cmdline.txt`, remove any `console=serial0,115200` (or `console=ttyAMA0,…`)
   entry; leave the rest of the line unchanged.

3. If a `hciuart` service exists, disable it: `sudo systemctl disable hciuart`.

4. Give your user access to serial ports, then reboot:

   ```bash
   sudo usermod -aG dialout "$USER"
   sudo reboot
   ```

5. Check the device: `ls -l /dev/serial0` should point to `ttyAMA0`. If `/dev/serial0` does not
   exist on your image, use `/dev/ttyAMA0` and set the bridge's `port` parameter accordingly.

## Checking the link

With the firmware running and the robot powered:

```bash
ros2 launch robot_bringup robot.launch.py
ros2 topic hz /balance/state      # about 50 Hz
ros2 topic echo /diagnostics      # "link: receiving"
```

If `diagnostics` reports corrupted frames, check that both sides use 460 800 Bd and that the
ground between ESP32 and Pi is connected.

## Flashing the ESP32 while it is wired to the Pi

UART0 is shared between the ESP32's USB port and the Pi. Stop the bridge before flashing over USB,
otherwise the Pi's TX line fights the USB-serial chip (see {doc}`../hardware/known-issues`).
