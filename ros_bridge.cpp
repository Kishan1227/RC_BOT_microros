#include <Arduino.h>
#include <math.h>

#include <WiFi.h>
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include <geometry_msgs/msg/twist.h>
#include <geometry_msgs/msg/vector3.h>
#include <std_msgs/msg/float32.h>
#include <nav_msgs/msg/odometry.h>

#include "config.h"
#include "state.h"
#include "encoders.h"
#include "motors.h"
#include "autonomous.h"
#include "ros_bridge.h"

// ---------------------------------------------------------------------
// ROS interface (all in ROS conventions: metres, radians, yaw CCW+)
//
//   sub  /cmd_vel          geometry_msgs/Twist    linear.x, angular.z
//   sub  /rc_bot/goal_xyz  geometry_msgs/Vector3  x fwd [m], y left [m],
//                                                 z final yaw [rad]
//                          (same shortest-path X/Y/Z move as the web page,
//                           all relative to the pose at the moment received)
//   sub  /rc_bot/turn      std_msgs/Float32       relative turn [rad], CCW+
//   pub  /odom             nav_msgs/Odometry      encoder dead-reckoning
// ---------------------------------------------------------------------

enum RosState { WAITING_AGENT, AGENT_AVAILABLE, AGENT_CONNECTED, AGENT_DISCONNECTED };
static RosState rosState = WAITING_AGENT;

static rclc_support_t support;
static rcl_allocator_t allocator;
static rcl_node_t node;
static rclc_executor_t executor;
static rcl_publisher_t odomPub;
static rcl_subscription_t cmdSub, goalSub, turnSub;

static geometry_msgs__msg__Twist cmdMsg;
static geometry_msgs__msg__Vector3 goalMsg;
static std_msgs__msg__Float32 turnMsg;
static nav_msgs__msg__Odometry odomMsg;

// latest /cmd_vel
static volatile float rosV = 0.0f;   // m/s
static volatile float rosW = 0.0f;   // rad/s (ROS sign)
static volatile bool rosCmdActive = false;
static volatile unsigned long lastRosCmdTime = 0;

// odometry state
static float odomX = 0, odomY = 0, odomYaw = 0;
static long odomPrevL = 0, odomPrevR = 0;
static unsigned long lastOdom = 0, lastPing = 0, lastRosControl = 0;

static const float DEG2RAD_F = 0.017453292519943295f;
static const float RAD2DEG_F = 57.29577951308232f;

// ---------------------------------------------------------------------
// Subscription callbacks
// ---------------------------------------------------------------------
static void cmdVelCallback(const void *msgin) {
  const geometry_msgs__msg__Twist *m = (const geometry_msgs__msg__Twist *)msgin;
  rosV = (float)m->linear.x;
  rosW = (float)m->angular.z;
  lastRosCmdTime = millis();
  rosCmdActive = true;

  // /cmd_vel overrides any running autonomous move (a zero Twist = stop).
  if (autoState != AUTO_IDLE) {
    autoState = AUTO_IDLE;
    resetPID();
  }
}

static void goalCallback(const void *msgin) {
  const geometry_msgs__msg__Vector3 *m = (const geometry_msgs__msg__Vector3 *)msgin;

  // ROS (x fwd, y left, yaw CCW+) -> firmware (x fwd, y/heading positive
  // toward the README's "right"). YAW_SIGN handles the mirror.
  float xMm = (float)m->x * 1000.0f;
  float yMm = (float)m->y * 1000.0f * YAW_SIGN;
  requestedZ = (float)m->z * RAD2DEG_F * YAW_SIGN;

  rosCmdActive = false;
  fwd = bwd = left = right = false;
  startXY(xMm, yMm);
}

static void turnCallback(const void *msgin) {
  const std_msgs__msg__Float32 *m = (const std_msgs__msg__Float32 *)msgin;

  rosCmdActive = false;
  fwd = bwd = left = right = false;
  startTurn(m->data * RAD2DEG_F * YAW_SIGN);
}

// ---------------------------------------------------------------------
// /cmd_vel -> motors
// ---------------------------------------------------------------------
bool rosCommandActive() {
  return rosCmdActive && (millis() - lastRosCmdTime <= ROS_CMD_TIMEOUT_MS);
}

static int velToPwm(float v) {
  float p = v / MAX_WHEEL_SPEED_MPS * 255.0f;
  if (fabsf(p) < 1.0f) return 0;
  if (fabsf(p) < MIN_PWM) p = (p > 0) ? MIN_PWM : -MIN_PWM;
  return (int)constrain(p, -255.0f, 255.0f);
}

void updateRosMotors() {
  unsigned long now = millis();
  if (now - lastRosControl > 2UL * CONTROL_INTERVAL_MS) resetPID();  // fresh start
  lastRosControl = now;

  float v = rosV;
  float wFw = rosW * YAW_SIGN;                    // firmware heading rate [rad/s]
  float halfBase = (WHEEL_BASE_MM / 1000.0f) / 2.0f;

  // firmware: heading = (distR - distL) / base  =>  wFw = (vR - vL) / base
  float vL = v - wFw * halfBase;
  float vR = v + wFw * halfBase;

  int speedL = velToPwm(vL);
  int speedR = velToPwm(vR);

  bool straight = fabsf(rosW) < 0.02f && fabsf(v) > 0.01f;
  if (straight) {
    applyStraightPID(speedL, speedR);   // encoder straight-line correction
  } else {
    resetPID();
  }
  setMotor(speedL, speedR);
}

// ---------------------------------------------------------------------
// /odom
// ---------------------------------------------------------------------
static void publishOdom(unsigned long now) {
  if (now - lastOdom < ODOM_INTERVAL_MS) return;
  float dt = (now - lastOdom) / 1000.0f;
  lastOdom = now;

  long lt = leftTicks, rt = rightTicks;
  float dL = ((float)(lt - odomPrevL) / LEFT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / 1000.0f;
  float dR = ((float)(rt - odomPrevR) / RIGHT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM / 1000.0f;
  odomPrevL = lt;
  odomPrevR = rt;

  float dc = (dL + dR) / 2.0f;
  float yaw = YAW_SIGN * getHeadingDeg() * DEG2RAD_F;   // continuous, not wrapped
  float yawMid = (odomYaw + yaw) / 2.0f;

  odomX += dc * cosf(yawMid);
  odomY += dc * sinf(yawMid);
  float wz = (yaw - odomYaw) / dt;
  odomYaw = yaw;

  int64_t t = rmw_uros_epoch_nanos();     // 0 if time sync with agent failed
  odomMsg.header.stamp.sec = (int32_t)(t / 1000000000LL);
  odomMsg.header.stamp.nanosec = (uint32_t)(t % 1000000000LL);

  odomMsg.pose.pose.position.x = odomX;
  odomMsg.pose.pose.position.y = odomY;
  odomMsg.pose.pose.position.z = 0.0;
  odomMsg.pose.pose.orientation.x = 0.0;
  odomMsg.pose.pose.orientation.y = 0.0;
  odomMsg.pose.pose.orientation.z = sinf(yaw / 2.0f);
  odomMsg.pose.pose.orientation.w = cosf(yaw / 2.0f);

  odomMsg.twist.twist.linear.x = dc / dt;
  odomMsg.twist.twist.angular.z = wz;

  (void)rcl_publish(&odomPub, &odomMsg, NULL);
}

// ---------------------------------------------------------------------
// micro-ROS entity lifecycle (reconnects automatically if the agent
// restarts or Wi-Fi drops)
// ---------------------------------------------------------------------
static bool createEntities() {
  allocator = rcl_get_default_allocator();

  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) return false;
  if (rclc_node_init_default(&node, "rc_bot", "", &support) != RCL_RET_OK) return false;

  if (rclc_publisher_init_best_effort(
        &odomPub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(nav_msgs, msg, Odometry), "odom") != RCL_RET_OK) return false;

  // best-effort subscribers accept both reliable and best-effort publishers
  if (rclc_subscription_init_best_effort(
        &cmdSub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Twist), "cmd_vel") != RCL_RET_OK) return false;
  if (rclc_subscription_init_best_effort(
        &goalSub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(geometry_msgs, msg, Vector3), "rc_bot/goal_xyz") != RCL_RET_OK) return false;
  if (rclc_subscription_init_best_effort(
        &turnSub, &node, ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Float32), "rc_bot/turn") != RCL_RET_OK) return false;

  if (rclc_executor_init(&executor, &support.context, 3, &allocator) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &cmdSub, &cmdMsg, &cmdVelCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &goalSub, &goalMsg, &goalCallback, ON_NEW_DATA) != RCL_RET_OK) return false;
  if (rclc_executor_add_subscription(&executor, &turnSub, &turnMsg, &turnCallback, ON_NEW_DATA) != RCL_RET_OK) return false;

  rmw_uros_sync_session(1000);            // time sync for /odom stamps

  odomPrevL = leftTicks;
  odomPrevR = rightTicks;
  lastOdom = millis();
  return true;
}

static void destroyEntities() {
  rmw_context_t *rmwContext = rcl_context_get_rmw_context(&support.context);
  (void)rmw_uros_set_context_entity_destroy_session_timeout(rmwContext, 0);

  (void)rcl_publisher_fini(&odomPub, &node);
  (void)rcl_subscription_fini(&cmdSub, &node);
  (void)rcl_subscription_fini(&goalSub, &node);
  (void)rcl_subscription_fini(&turnSub, &node);
  (void)rclc_executor_fini(&executor);
  (void)rcl_node_fini(&node);
  (void)rclc_support_fini(&support);
}

// ---------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------
void setupRos() {
  // Static message strings (Odometry has two string fields)
  odomMsg.header.frame_id.data = (char *)"odom";
  odomMsg.header.frame_id.size = 4;
  odomMsg.header.frame_id.capacity = 5;
  odomMsg.child_frame_id.data = (char *)"base_link";
  odomMsg.child_frame_id.size = 9;
  odomMsg.child_frame_id.capacity = 10;

  // Same UDP transport that set_microros_wifi_transports() installs, minus its
  // WiFi.begin(): the ESP32 is the access point (ROBOT_ESP32), it cannot
  // join its own network as a station.
  static struct micro_ros_agent_locator locator;
  locator.address.fromString(ROS_AGENT_IP);
  locator.port = ROS_AGENT_PORT;

  rmw_uros_set_custom_transport(
    false,
    (void *)&locator,
    arduino_wifi_transport_open,
    arduino_wifi_transport_close,
    arduino_wifi_transport_write,
    arduino_wifi_transport_read);

  Serial.print("micro-ROS agent expected at ");
  Serial.print(ROS_AGENT_IP);
  Serial.print(":");
  Serial.println(ROS_AGENT_PORT);
}

void spinRos() {
  unsigned long now = millis();

  switch (rosState) {
    case WAITING_AGENT:
      if (now - lastPing > 500) {
        lastPing = now;
        if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) rosState = AGENT_AVAILABLE;
      }
      break;

    case AGENT_AVAILABLE:
      if (createEntities()) {
        rosState = AGENT_CONNECTED;
        Serial.println("micro-ROS: connected");
      } else {
        rosState = WAITING_AGENT;
      }
      break;

    case AGENT_CONNECTED:
      if (now - lastPing > 1000) {
        lastPing = now;
        if (rmw_uros_ping_agent(50, 3) != RMW_RET_OK) {
          rosState = AGENT_DISCONNECTED;
          break;
        }
      }
      (void)rclc_executor_spin_some(&executor, RCL_MS_TO_NS(2));
      publishOdom(now);
      break;

    case AGENT_DISCONNECTED:
      Serial.println("micro-ROS: agent lost");
      rosCmdActive = false;        // motors stop via the normal timeout path
      destroyEntities();
      rosState = WAITING_AGENT;
      break;
  }
}
