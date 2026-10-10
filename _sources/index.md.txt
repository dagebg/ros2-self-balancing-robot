# ROS2 Self-Balancing Robot

A two-wheeled, self-balancing differential-drive robot built from scratch: a 3D-printed chassis,
an ESP32 running the real-time balance loop, and a Raspberry Pi 4 running ROS 2 Jazzy as the
high-level layer.

- **Source:** [github.com/dagebg/ros2-self-balancing-robot](https://github.com/dagebg/ros2-self-balancing-robot)
- **Start here:** {doc}`overview/architecture`, then the {doc}`overview/roadmap`.

:::{toctree}
:maxdepth: 2
:caption: Overview

overview/architecture
overview/roadmap
:::

:::{toctree}
:maxdepth: 2
:caption: Hardware

hardware/index
:::

:::{toctree}
:maxdepth: 2
:caption: Deployment

deployment/networking
:::

:::{toctree}
:maxdepth: 2
:caption: Development

development/setup
development/contributing
development/testing
development/ci-cd
development/adr/index
:::

:::{toctree}
:maxdepth: 1
:caption: Reference

api/index
changelog
:::
