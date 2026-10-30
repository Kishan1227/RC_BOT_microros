#pragma once

void setupRos();            // join Wi-Fi + configure micro-ROS transport
void spinRos();             // call every loop(): agent link, subscriptions, /odom
bool rosCommandActive();    // true while a fresh /cmd_vel is driving the robot
void updateRosMotors();     // turn the latest /cmd_vel into motor PWM
