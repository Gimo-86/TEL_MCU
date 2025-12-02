#pragma once
#include <Arduino.h>

class IMU {
public:
    bool begin();
    void update();
    float getPitch();
    float getRoll();
    float getYaw();

    void readMPU();
    void computeAngles(float dt);
    

private:
    float pitch = 0, roll = 0, yaw = 0;
    unsigned long lastTime = 0;
    int16_t ax=0, ay=0, az=0;
    int16_t gx=0, gy=0, gz=0;

    // Cascaded 1st-order filter states for accelerometer-based angle
    float accPitch_f1 = 0.0f;
    float accPitch_f2 = 0.0f;
    float accRoll_f1  = 0.0f;
    float accRoll_f2  = 0.0f;
    // Low-pass filter state for gyro Z (yaw rate)
    float gz_f = 0.0f;
    // cutoff frequency for gyro-z low-pass (Hz)
    float gyroZCutoffHz = 10.0f;
};
