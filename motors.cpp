#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "motors.h"

void setMotor(int speedL, int speedR) {
  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);

  if (speedL >= 0) {
    PWM_WRITE(L_IN1, CH_L_IN1, speedL);
    PWM_WRITE(L_IN2, CH_L_IN2, 0);
  } else {
    PWM_WRITE(L_IN1, CH_L_IN1, 0);
    PWM_WRITE(L_IN2, CH_L_IN2, -speedL);
  }

  if (speedR >= 0) {
    PWM_WRITE(R_IN1, CH_R_IN1, speedR);
    PWM_WRITE(R_IN2, CH_R_IN2, 0);
  } else {
    PWM_WRITE(R_IN1, CH_R_IN1, 0);
    PWM_WRITE(R_IN2, CH_R_IN2, -speedR);
  }
}

void resetPID() {
  pidIntegral = 0;
  pidLastError = 0;
  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;
}

void applyStraightPID(int &speedL, int &speedR) {
  long dLeft = leftTicks - prevLeftTicks;
  long dRight = rightTicks - prevRightTicks;

  prevLeftTicks = leftTicks;
  prevRightTicks = rightTicks;

  float leftRevolutions = (float)dLeft / LEFT_TICKS_PER_REV;
  float rightRevolutions = (float)dRight / RIGHT_TICKS_PER_REV;

  float error = leftRevolutions - rightRevolutions;

  pidIntegral += error;
  pidIntegral = constrain(pidIntegral, -100.0f, 100.0f);

  float derivative = error - pidLastError;

  float correction =
    Kp * error +
    Ki * pidIntegral +
    Kd * derivative;

  pidLastError = error;

  speedR = (int)(speedR * RIGHT_MOTOR_SCALE);

  speedL -= (int)correction;
  speedR += (int)correction;

  speedL = constrain(speedL, -255, 255);
  speedR = constrain(speedR, -255, 255);
}

void updateManualMotors() {
  bool moving = fwd || bwd;
  bool turning = left || right;

  int speedL = 0;
  int speedR = 0;

  if (moving) {
    int dir = fwd ? 1 : -1;                           // FWD/BACK + L/R code present hire

    speedL = BASE_SPEED * dir;
    speedR = BASE_SPEED * dir;

    if (turning) {
      int off = left ? -TURN_OFFSET : TURN_OFFSET;
      speedL += off * dir;
      speedR -= off * dir;
    }
  } else if (turning) {                               
    int dir = left ? -1 : 1;
    speedL = -PIVOT_SPEED * dir;
    speedR = PIVOT_SPEED * dir;
  }

  if (moving && !turning) {
    applyStraightPID(speedL, speedR);
  } else {
    resetPID();
  }

  setMotor(speedL, speedR);
}
