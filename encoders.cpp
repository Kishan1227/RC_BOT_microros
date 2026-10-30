#include <Arduino.h>
#include "config.h"
#include "state.h"
#include "encoders.h"

void IRAM_ATTR leftISR() {
  bool c1 = digitalRead(L_C1);
  bool c2 = digitalRead(L_C2);
  if (c1 == c2) leftTicks++;
  else leftTicks--;
}

void IRAM_ATTR rightISR() {
  bool c1 = digitalRead(R_C1);
  bool c2 = digitalRead(R_C2);
  if (c1 == c2) rightTicks++;
  else rightTicks--;
}

float getLeftDistance() {
  return ((float)leftTicks / LEFT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
}

float getRightDistance() {
  return ((float)rightTicks / RIGHT_TICKS_PER_REV) * WHEEL_CIRCUMFERENCE_MM;
}

float getHeadingDeg() {
  float distL = getLeftDistance();
  float distR = getRightDistance();
  float headingRad = (distR - distL) / WHEEL_BASE_MM;
  return headingRad * 180.0f / PI;
}
