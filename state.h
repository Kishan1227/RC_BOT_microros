#pragma once
#include <Arduino.h>

enum AutoState {
  AUTO_IDLE,
  AUTO_TURN,
  AUTO_TURN_TO_GOAL,
  AUTO_MOVE_GOAL,
  AUTO_FINAL_TURN,
  AUTO_DONE,
  AUTO_MOVE_X,
  AUTO_MOVE_Y,
  AUTO_TURN_X,
  AUTO_TURN_Y
};

extern volatile long leftTicks;
extern volatile long rightTicks;

extern long prevLeftTicks;
extern long prevRightTicks;

extern bool fwd;
extern bool bwd;
extern bool left;
extern bool right;

extern unsigned long lastCmdTime;
extern unsigned long lastControlTime;

extern AutoState autoState;

// X/Y/Z command
extern float requestedX;
extern float requestedY;
extern float requestedZ;

// Shortest straight-line path
extern float goalDistance;
extern float goalTravelAngle;
extern float goalTargetHeading;

// Final Z heading
extern float finalTargetHeading;

extern long autoStartLeftTicks;
extern long autoStartRightTicks;
extern float autoStartHeading;

extern float autoTargetHeading;
extern float targetTurnAngle;

extern int autoMoveDirection;

extern float pidIntegral;
extern float pidLastError;
