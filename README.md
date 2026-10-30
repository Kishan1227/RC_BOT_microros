
# ESP32 Robot X/Y/Z Shortest-Path Control
=======
# RC_ROBO 

`ESP32 Robot Control Project`

This project is an ESP32-based two-wheel robot controlled through a web page.
The robot can be controlled manually using buttons or can perform programmed movements such as:
Forward and backward movement
-  Left and right turns
-  Turning by a specific angle
-  Moving a specific distance in X/Y directions
-  Encoder-based distance measurement
-  Encoder-based heading measurement
-  Straight-line PID correction
-  Real-time robot status through a web page
The ESP32 creates its own Wi-Fi network, so the robot can be controlled directly from a phone or computer without needing an external Wi-Fi router. The original project uses the SSID ROBOT_ESP32.


This project controls a two-wheel ESP32 robot from a web page.

## X/Y/Z movement

<<<<<<< HEAD
The robot now treats X and Y as a target point measured from the position where the command starts. Z is the final heading.

Example:

```text
X = 100 mm
Y = 150 mm
Z = 60°
```

The robot calculates the shortest straight-line path using trigonometry:

```text
Distance = sqrt(X² + Y²)
Angle    = atan2(Y, X)
```

It then:

1. Saves the starting heading.
2. Calculates the angle to the target with `atan2(Y, X)`.
3. Calculates the shortest distance with the Pythagorean theorem.
4. Turns through the shortest angular direction to face the target.
5. Drives straight along the hypotenuse to the target.
6. Uses encoder feedback and PID correction to stay straight.
7. After reaching the target, turns to Z.
8. Stops.

Z is relative to the heading at the beginning of the command.

Examples when the starting heading is 0°:

- X=100, Y=0, Z=0 → drive 100 mm and finish at 0°
- X=0, Y=100, Z=0 → turn 90° right, drive 100 mm, turn back to 0°
- X=100, Y=100, Z=90 → turn 45° right, drive about 141.4 mm, then finish at 90°
- X=-100, Y=0, Z=0 → turn 180°, drive 100 mm, then finish at 0°

`atan2()` is used instead of `atan(Y/X)` because it correctly handles all quadrants and the case where X is zero.

## Project files

- `robot.ino` — main setup and loop
- `config.h` — robot configuration
- `state.h` — shared robot state
- `encoders.cpp/.h` — encoder and heading calculations
- `motors.cpp/.h` — motor and PID control
- `autonomous.cpp/.h` — shortest-path X/Y movement and final Z turn
- `web.cpp/.h` — web interface and HTTP handlers

Keep all files in the same Arduino/PlatformIO project folder.
=======
Keep all files in the same Arduino/PlatformIO project directory. The code intentionally preserves the behavior and constants of the supplied sketch rather than redesigning the control logic.

`How to Use the Robot`

1. Turn on the ESP32
The ESP32 creates its own Wi-Fi network.
Connect your phone or computer to:
Wi-Fi: `ROBOT_ESP32`
Password: `roboy1234`
The ESP32 then prints its IP address through the Serial Monitor. 
Pasted code.cpp
2. Open the Robot Control Page
Enter the ESP32's IP address in your browser.
The robot control page will appear.
>>>>>>> cf486fc0d4cb7a4148408fe8a00ac9490825bde5
