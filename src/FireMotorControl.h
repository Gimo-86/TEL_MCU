#pragma once
#include <Arduino.h>

// Control two firing motors (on/off via PWM)
void initFireMotors();
void setFireMotors(bool on);
