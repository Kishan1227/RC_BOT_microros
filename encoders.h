#pragma once
void IRAM_ATTR leftISR();
void IRAM_ATTR rightISR();
float getLeftDistance();
float getRightDistance();
float getHeadingDeg();
