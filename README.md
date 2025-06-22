# elrs_teleop

`elrs_teleop` is a ROS 2 package that enables teleoperation of ground robots using ExpressLRS (ELRS) RC transmitters and receivers. It translates RC switch and stick inputs into ROS messages such as `cmd_vel` and custom motion mode topics.

## Features

- Decodes ELRS RC signals into ROS 2 messages
- Publishes velocity commands to `/cmd_vel`
- Supports motion mode switching (e.g., manual, explore, goto)
- Sends telemetry feedback to the transmitter (battery voltage, CPU usage, coordinates, etc.)
- Supports custom LUA display for real-time robot status

## Telemetry

The system optionally sends telemetry data back to the ELRS transmitter, enabling real-time monitoring through a custom LUA script. Supported telemetry values include:

- Battery voltage
- CPU usage percentage
- Robot coordinates (x, y)
- Current motion mode
- Other custom values (e.g., heading, velocity)

## Topics

**Subscribed:**
- `/rc_channels` — Receives raw channel data from ELRS receiver

**Published:**
- `/cmd_vel` — Velocity commands for the robot
- `/motion_mode` — Motion mode selector (manual, explore, etc.)

## Launch

```bash
ros2 launch elrs_teleop elrs_teleop_launch.py
```