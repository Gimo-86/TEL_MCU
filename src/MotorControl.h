#ifndef MOTORCONTROL_H
#define MOTORCONTROL_H

#include <Arduino.h>
#include "PIDController.h"

// Forward declaration for interrupt handlers
class Motor;
extern Motor* motorInstances[4];

// ISR handlers (static functions)
void motorISR0();
void motorISR1();
void motorISR2();
void motorISR3();

class Motor {
  public:
    Motor(int pwmPin, int in1, int in2, int enca, float kp, float ki, float kd);

    void begin();
    void setTargetRPM(float target);
    void updateEncoder();        // ISR callback for encoder
    void updateControl();

    // Functions for debugging
    float getRPM();
    int getPWM();
    float getTargetRPM();
    long getPulseCount();

    int minPWM = 15;
    float maxRPM = 300;  

  private:
    int _pwmPin, _in1, _in2, _enca;
    volatile long pulseCount;
    float rpm;
    unsigned long lastTime;
    long lastCount;
    int pwmValue;
    float targetRPM;
    PIDController _pid;
};

#endif