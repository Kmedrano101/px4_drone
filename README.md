**Languages:** [English](README.md) | [Español](docs/README.es.md)

# px4_drone

🛸 ROS 2 (C++/Python) package for **PX4 Offboard control** from a Raspberry Pi
companion computer — arming/mode sequencing, failsafe verification, and
takeoff→hold→land flight, talking to the flight controller over
**uXRCE-DDS** on a serial link. Built to work across **multiple PX4
firmware versions** (v1.14 and v1.17) without touching C++ code.

> 📓 Companion engineering journal (flight reports, `.ulog` diagnostics,
> firmware backups): [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev)

## Table of Contents

- [Overview](#overview)
- [Installation](#installation)
- [Usage](#usage)
- [Project Resources](#project-resources)
- [Contributing](#contributing)
- [License](#license)
- [Contact](#contact)

## Overview

**Stack**
- 🚁 **FC:** PX4 — tested on v1.14 (`release/1.14`) and v1.17 (HKUST_NXT_DUAL)
- 💻 **Companion:** Raspberry Pi 4 · Ubuntu 24.04 · ROS 2 Jazzy
- 🔗 **Link:** Micro XRCE-DDS Agent, serial `/dev/ttyAMA0` @ 921600 baud
- 📷 **Camera:** official Raspberry Pi camera via `camera_ros`/`libcamera`

**Nodes**
| Node | Purpose |
|---|---|
| `offboard_control` | Arms in OFFBOARD (attitude, zero thrust, never takes off), then simulates heartbeat loss to verify the FC's failsafe reaction. |
| `takeoff_position_hold_indoor` | Waits for a valid optical-flow/LiDAR position estimate, arms, takes off, holds position, lands — fully automatic. |
| `takeoff_position_hold_outdoor` | Same state machine, gated on a 3D GPS fix instead. |
| `offboard_streamer.py` | Minimal Python setpoint streamer for bring-up/link testing. |

**Milestones**
- ✅ Offboard link RPi ⇄ FC established and diagnosed end-to-end (`px4_msgs` alignment, `MicroXRCEAgent` Fast-DDS/Fast-CDR version match, DDS domain match)
- ✅ Failsafe test (arm → hold → simulated heartbeat loss → confirmed disarm) verified on **two different FCs**
- ✅ Multi-firmware support: switch FC by checking out a `px4_msgs` branch + one launch parameter, no code changes
- ✅ Official Raspberry Pi camera bring-up (`camera_ros`, fixed an upstream `libcamera` bug for the OV5647 sensor)
- 🔄 Indoor position hold — blocked on EKF2/sensor tuning, tracked in [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev)
- ⏳ First outdoor GPS position-hold flight test

## Installation

`px4_msgs` is vendored as a **git submodule pinned to the exact commit this
package was built against** (`px4_msgs/`, currently the v1.14 firmware
commit — see [Usage](#usage) to switch FCs). Clone with
`--recurse-submodules`, and symlink it up to the workspace `src/` level so
`colcon` (which doesn't recurse into an already-found package) discovers it
alongside `px4_drone`:

```bash
cd ~/drone_ws/src
git clone --recurse-submodules https://github.com/Kmedrano101/px4_drone.git
ln -s px4_drone/px4_msgs px4_msgs

cd ~/drone_ws
colcon build --packages-select px4_msgs px4_drone --symlink-install
source install/setup.bash
```

Also needs a built [`MicroXRCEAgent`](https://github.com/eProsima/Micro-XRCE-DDS-Agent)
on the Raspberry Pi (see [Project Resources](#project-resources) for the
exact version/flags that actually interoperate with ROS 2 Jazzy — the
default `main` build does **not**).

## Usage

```bash
# Failsafe verification (safe: never takes off, zero thrust)
ros2 launch px4_drone offboard_control_cpp.launch.py

# Full takeoff + hold + land — REQUIRES confirm_takeoff:=true (props spin for real)
ros2 launch px4_drone takeoff_position_hold_indoor.launch.py confirm_takeoff:=true

# Raspberry Pi camera
ros2 launch px4_drone camera.launch.py
```

⚠️ **Safety:** any node that arms the FC spins the motors, even at zero
thrust — secure or remove propellers before running `offboard_control`, and
double-check the physical space before `confirm_takeoff:=true`.

**Targeting a different FC firmware** (default: v1.14, no message-version suffix):

```bash
cd ~/drone_ws/src/px4_msgs && git checkout fc-v17-82e3322e   # or fc-v14-ffb6e80
cd ~/drone_ws && colcon build --packages-select px4_msgs px4_drone
ros2 launch px4_drone offboard_control_cpp.launch.py topic_version_suffix:=_v1
```

## Project Resources

**Documentation** (`docs/`)
- [`offboard_control` — RPi ⇄ FC communication](docs/offboard_control.md) — architecture, message flow, failsafe behavior, and the full troubleshooting log (`px4_msgs`/`px4_msgs` alignment, `MicroXRCEAgent` build, DDS domain, multi-FC setup)

**Related**
- [px4_drone_dev](https://github.com/Kmedrano101/px4_drone_dev) — flight reports, `.ulog` analysis tooling, firmware backups
- [PX4/px4_msgs](https://github.com/PX4/px4_msgs) — message definitions (this repo depends on specific branches per FC firmware, see above)

## Contributing

Personal R&D project — issues and suggestions welcome. See
[docs/offboard_control.md](docs/offboard_control.md) for the current
architecture and open troubleshooting notes before proposing changes.

## License

Distributed under the [Apache License 2.0](LICENSE).

## Contact

Kevin Medrano — [kevin.ejem18@gmail.com](mailto:kevin.ejem18@gmail.com) · [@Kmedrano101](https://github.com/Kmedrano101)
