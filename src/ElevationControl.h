#pragma once
#include <Arduino.h>

// Simple elevation motor control driven by CH3 (ch[CH_ELEVATE]).
// Behavior:
//  - ch > NEUTRAL + deadzone  -> increase angle (DIR HIGH)
//  - ch within deadzone       -> stop
//  - ch < NEUTRAL - deadzone  -> decrease angle (DIR LOW)

void initElevation();
// Call every loop with raw SBUS channel value (0..2047 approx)
void setElevationFromChannel(uint16_t chValue);
// Immediate stop
void stopElevationMotor();
