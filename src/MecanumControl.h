#pragma once
#include <Arduino.h>
#include "MotorControl.h"

class MotorSystem {
public:
    MotorSystem();

    Motor FL;
    Motor FR;
    Motor RL;
    Motor RR;
    
    void initMecanum();
    void mecanumDrive(int ch2, int ch4, int chRotate);
    void updateAll(float dt);
    // Directly set target RPMs for each motor (used by autopilot)
    void setTargetRPMs(float m1, float m2, float m3, float m4);
    void stop();
    
    
};