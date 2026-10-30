#pragma once
void setMotor(int speedL, int speedR);
void resetPID();
void applyStraightPID(int &speedL, int &speedR);
void updateManualMotors();
