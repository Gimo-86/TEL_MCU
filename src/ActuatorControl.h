#pragma once
#include <Arduino.h>

// Actuator control (non-blocking)
// Uses: ACTUATOR_PWM_PIN (PWM), ACTUATOR_DIR_PIN (direction)
// Limit switches: ACT_LIMIT_EXT_PIN (extended), ACT_LIMIT_RET_PIN (retracted)

void initActuator();
bool startFireCycle();       // start one retract->extend cycle; returns true if started
void updateActuator();       // call every loop to progress state machine
bool actuatorIsBusy();       // true while cycle in progress
void actuatorStopMotor();    // immediate motor off
const char* actuatorLastError();
