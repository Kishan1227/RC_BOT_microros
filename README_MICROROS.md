# RC_BOT with micro-ROS

The original ESP32 firmware (web page, encoders, PID, X/Y/Z move) is kept. micro-ROS is added
alongside it in `robot/ros_bridge.cpp`, so the robot is now a ROS 2 node named `rc_bot`.

| Direction | Topic | Type | Meaning |
|---|---|---|---|
| sub | `/cmd_vel` | `geometry_msgs/Twist` | `linear.x` m/s, `angular.z` rad/s. Zero Twist stops the robot. Stops by itself after 500 ms of silence. |
| sub | `/rc_bot/goal_xyz` | `geometry_msgs/Vector3` | x forward [m], y left [m], z final yaw [rad]. Same shortest-path move as the web page, relative to the current pose. |
| sub | `/rc_bot/turn` | `std_msgs/Float32` | Relative turn in rad, counter-clockwise positive. |
| pub | `/odom` | `nav_msgs/Odometry` | Encoder dead-reckoning at 20 Hz (frame `odom` -> `base_link`). |

The web page still works. Its buttons take priority over `/cmd_vel`.

## 1. Wi-Fi (same as the original project)
No Wi-Fi credentials to enter. The ESP32 keeps its own network from `robot.ino`:
SSID `ROBOT_ESP32`, password `roboy1234`.
- Connect the ROS 2 PC to `ROBOT_ESP32`.
- In `robot/config.h`, set `ROS_AGENT_IP` to the PC's address on that network. The ESP32 is `192.168.4.1`
  and the first client normally gets `192.168.4.2` (the default). Verify with `ip a`.
- **Calibrate** `MAX_WHEEL_SPEED_MPS` (wheel speed at PWM 255). 0.30 is a placeholder.
- The PC has no internet while on this network (docker may need the image pulled beforehand).

## 2. Arduino setup
- Install the `micro_ros_arduino` library whose branch matches your ROS 2 distro (Humble, Jazzy, ...).
- Board: ESP32 Dev Module. micro_ros_arduino documents ESP32 core 2.0.x; the PWM code now
  compiles on both core 2.x and 3.x, but micro_ros_arduino on core 3.x is untested by me.

## 3. Run the agent on the PC (same distro as the library)
```bash
docker run -it --rm --net=host microros/micro-ros-agent:humble udp4 --port 8888 -v6
```

## 4. Test
```bash
ros2 topic echo /odom
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 topic pub --once /rc_bot/turn std_msgs/msg/Float32 "{data: 1.5708}"
ros2 topic pub --once /rc_bot/goal_xyz geometry_msgs/msg/Vector3 "{x: 0.10, y: 0.15, z: 1.047}"
```
A positive turn must rotate the robot **left** and make `/odom` yaw increase. If it is mirrored,
set `YAW_SIGN` to `1.0f` in `config.h`.

## Notes
- ROS and the web page both use the `ROBOT_ESP32` access point (web page at 192.168.4.1).
- The agent can start later or restart; the firmware reconnects automatically.
- The original `README.md` contains unresolved git merge markers (`<<<<<<<`, `>>>>>>>`). Clean it up before committing.
