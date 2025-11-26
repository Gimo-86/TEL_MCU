#pragma once
#include <Arduino.h>

class MotorSystem {
public:
    MotorSystem();

    void initMecanum();
    void mecanumDrive(int ch2, int ch4, int chRotate);
    
private:
    Motor FL;
    Motor FR;
    Motor RL;
    Motor RR;
};