#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "PIDController.h"

class Motor {
  public:
    Motor(int pwmPin, int in1, int in2, int enca, int encb, float kp, float ki, float kd);

    void begin();
    void setTargetRPM(float target);
    void updateEncoder();
    void updateControl();
    void updateEncoderR();
    float getRPM();
    int getPWM();
    float getTargetRPM();

    int minPWM = 15;
    float maxRPM = 120;  // 你可依實測修改

  private:
    int _pwmPin, _in1, _in2, _enca, _encb;
    volatile long pulseCount;
    volatile int rotationDir;
    float rpm;
    unsigned long lastTime;
    long lastCount;
    int pwmValue;

    float targetRPM;
    PIDController _pid;
};

#endif