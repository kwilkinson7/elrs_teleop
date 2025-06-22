# elrs_teleop

`elrs_teleop` is a ROS 2 package that enables teleoperation of ground robots using ExpressLRS (ELRS) RC transmitters and receivers. It translates RC switch and stick inputs into ROS messages such as `cmd_vel` and custom motion mode topics.

## Features

- Decodes ELRS RC signals into ROS 2 messages
- Publishes velocity commands to `/cmd_vel`
- Supports motion mode switching (e.g., manual, explore, goto)

## Topics

**Subscribed:**
- `/battery_voltage` 

**Published:**
- `/cmd_vel` — Velocity commands for the robot
- `/motion_mode` — Motion mode selector (manual, explore, etc.)

## Launch

```bash
ros2 launch elrs_teleop elrs_teleop_launch.py
```
## Future Improvements

- Improve current motion mode
- Sends telemetry feedback to the transmitter (battery voltage, CPU usage, coordinates, heading, velocity)
- Supports custom LUA display for real-time robot status