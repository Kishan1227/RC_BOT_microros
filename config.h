#pragma once
#include <Arduino.h>

extern const char* ssid;
extern const char* password;

#define L_C1 16
#define L_C2 17
#define R_C1 18
#define R_C2 19

#define L_IN1 32
#define L_IN2 33
#define R_IN1 25
#define R_IN2 26

#define PWM_FREQ 5000
#define PWM_RES 8

#define WHEEL_CIRCUMFERENCE_MM 100.53f   // circumference of the actual wheel
#define WHEEL_BASE_MM 130.50f            // changed according to requirment
#define LEFT_TICKS_PER_REV 2020.0f        //manually check for individual motors
#define RIGHT_TICKS_PER_REV 2000.0f

#define BASE_SPEED 200
#define TURN_OFFSET 50
#define PIVOT_SPEED 190

#define CONTROL_INTERVAL_MS 50
#define CMD_TIMEOUT_MS 1500

extern float Kp;
extern float Ki;
extern float Kd;
extern float RIGHT_MOTOR_SCALE;

// ---------------------------------------------------------------------
// PWM compatibility: micro_ros_arduino documents ESP32 Arduino core 2.x,
// the original sketch used the core 3.x LEDC API. Support both.
// ---------------------------------------------------------------------
#define CH_L_IN1 0
#define CH_L_IN2 1
#define CH_R_IN1 2
#define CH_R_IN2 3

#if defined(ESP_ARDUINO_VERSION_MAJOR) && ESP_ARDUINO_VERSION_MAJOR >= 3
  #define PWM_ATTACH(pin, ch) ledcAttach(pin, PWM_FREQ, PWM_RES)
  #define PWM_WRITE(pin, ch, v) ledcWrite(pin, v)
#else
  #define PWM_ATTACH(pin, ch) do { ledcSetup(ch, PWM_FREQ, PWM_RES); ledcAttachPin(pin, ch); } while (0)
  #define PWM_WRITE(pin, ch, v) ledcWrite(ch, v)
#endif

// ---------------------------------------------------------------------
// micro-ROS settings
// ---------------------------------------------------------------------
// The ESP32 hosts its own Wi-Fi (SSID/password in robot.ino: ROBOT_ESP32 / roboy1234).
// Connect the ROS 2 PC to that network, run micro_ros_agent there, and set
// ROS_AGENT_IP to the PC's address on it. The ESP32 itself is 192.168.4.1;
// the first client usually gets 192.168.4.2 (check with `ip a` / `ipconfig`).
#define ROS_AGENT_IP       "192.168.4.2"
#define ROS_AGENT_PORT     8888

#define ROS_CMD_TIMEOUT_MS 500      // stop if no /cmd_vel for this long
#define ODOM_INTERVAL_MS   50       // /odom rate (20 Hz)

// CALIBRATE: wheel speed (m/s) at PWM 255. Drive at PWM 255 over a measured
// distance and time it. 0.30 is only a placeholder.
#define MAX_WHEEL_SPEED_MPS 0.30f
#define MIN_PWM 60                  // below this the motors stall

// Firmware heading is positive in the direction the README calls "right".
// ROS yaw is positive counter-clockwise (left). So yaw = -heading.
// If /odom yaw or turning direction is mirrored on your robot, flip to +1.
#define YAW_SIGN -1.0f
