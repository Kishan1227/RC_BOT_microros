#include <Arduino.h>
#include <math.h>
#include "config.h"
#include "state.h"
#include "encoders.h"
#include "motors.h"
#include "autonomous.h"

static float normalizeAngle(float angle) {
  while (angle > 180.0f) angle -= 360.0f;
  while (angle < -180.0f) angle += 360.0f;
  return angle;
}

void startTurn(float degrees) {
  autoStartHeading = getHeadingDeg();
  targetTurnAngle = degrees;
  autoTargetHeading = autoStartHeading + degrees;
  autoState = AUTO_TURN;
  resetPID();
}

void startXY(float x, float y) {
  requestedX = x;
  requestedY = y;

  autoStartHeading = getHeadingDeg();

  // ------------------------------------------------------------
  // TRIGONOMETRY
  // ------------------------------------------------------------
  // Shortest distance from start to (X,Y):
  //     distance = sqrt(X^2 + Y^2)
  //
  // Direction from start to goal:
  //     angle = atan2(Y, X)
  //
  // atan2 is used instead of atan(Y/X) because it correctly handles
  // all four quadrants and X = 0.
  // ------------------------------------------------------------
  goalDistance = sqrtf((x * x) + (y * y));

  goalTravelAngle = atan2f(y, x) * 180.0f / PI;

  // Goal direction is relative to the original heading.
  goalTargetHeading = autoStartHeading + goalTravelAngle;

  // Z is also relative to the original heading.
  finalTargetHeading = autoStartHeading + requestedZ;

  if (goalDistance < 0.5f) {
    // No movement needed. Just go to final Z heading.
    autoTargetHeading = finalTargetHeading;
    autoState = AUTO_FINAL_TURN;
    resetPID();
    return;
  }

  // Turn to the goal direction first.
  autoTargetHeading = goalTargetHeading;
  autoState = AUTO_TURN_TO_GOAL;
  resetPID();
}

void updateTurnToGoal() {
  float currentHeading = getHeadingDeg();
  float error = normalizeAngle(autoTargetHeading - currentHeading);

  // Goal direction reached.
  if (fabsf(error) <= 1.0f) {
    setMotor(0, 0);
    delay(100);

    autoStartLeftTicks = leftTicks;
    autoStartRightTicks = rightTicks;

    resetPID();
    autoMoveDirection = 1;
    autoState = AUTO_MOVE_GOAL;
    return;
  }

  // Turn using the shortest angular direction.
  int turnSpeed = 120;

  if (fabsf(error) < 15.0f)
    turnSpeed = 80;

  if (error > 0)
    setMotor(-turnSpeed, turnSpeed);   // right
  else
    setMotor(turnSpeed, -turnSpeed);   // left
}

void updateMoveGoal() {
  float leftDistance =
    ((float)(leftTicks - autoStartLeftTicks) / LEFT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;

  float rightDistance = ((float)(rightTicks - autoStartRightTicks) / RIGHT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;

  // Average wheel travel gives the distance travelled along the
  // straight path. Absolute values prevent one encoder sign from
  // making the distance negative.
  float travelled = (fabsf(leftDistance) + fabsf(rightDistance)) / 2.0f;

  if (travelled >= goalDistance) {
    setMotor(0, 0);
    delay(100);
    resetPID();

    // Position reached. Now rotate to the requested Z heading.
    autoTargetHeading = finalTargetHeading;
    autoState = AUTO_FINAL_TURN;
    return;
  }

  float remaining = goalDistance - travelled;

  int speed = BASE_SPEED;

  // Slow down near the target to reduce overshoot.
  if (remaining < 100.0f) speed = 120;
  if (remaining < 50.0f)  speed = 90;
  if (remaining < 20.0f)  speed = 65;

  int speedL = speed;
  int speedR = speed;

  // Keep the robot straight while travelling the hypotenuse.
  applyStraightPID(speedL, speedR);
  setMotor(speedL, speedR);
}

void updateFinalTurn() {
  float currentHeading = getHeadingDeg();
  float error = normalizeAngle(
    finalTargetHeading - currentHeading
  );

  if (fabsf(error) <= 1.0f) {
    setMotor(0, 0);
    resetPID();
    autoState = AUTO_DONE;
    return;
  }

  int turnSpeed = 120;

  if (fabsf(error) < 15.0f)
    turnSpeed = 80;

  // Turn through the shortest angular direction.
  if (error > 0)
    setMotor(-turnSpeed, turnSpeed);   // right
  else
    setMotor(turnSpeed, -turnSpeed);   // left
}

void updateAutonomous() {
  if (autoState == AUTO_TURN) {
    float currentHeading = getHeadingDeg();
    float error = normalizeAngle(
      autoTargetHeading - currentHeading
    );

    if (fabsf(error) <= 1.0f) {
      setMotor(0, 0);
      delay(100);
      resetPID();
      autoState = AUTO_DONE;
      return;
    }

    int turnSpeed =
      (fabsf(error) < 15.0f) ? 80 : 120;

    if (error > 0)
      setMotor(-turnSpeed, turnSpeed);
    else
      setMotor(turnSpeed, -turnSpeed);

  } else if (autoState == AUTO_TURN_TO_GOAL) {
    updateTurnToGoal();

  } else if (autoState == AUTO_MOVE_GOAL) {
    updateMoveGoal();

  } else if (autoState == AUTO_FINAL_TURN) {
    updateFinalTurn();

  } else if (autoState == AUTO_DONE) {
    setMotor(0, 0);
  }
}
