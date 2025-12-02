#pragma once
#include "MecanumControl.h"
#include "IMU.h"

class AutoDrive {
public:
    AutoDrive(MotorSystem* mec, IMU* imu);
    Motor* motorInstances[4];
    void start();
    void stop();
    void update();
    bool isRunning() const { return running; }

private:
    MotorSystem* mec;
    IMU* imu;

    bool running = false;
    int phase = 0;

    unsigned long phaseStart = 0;
    long startCounts[4] = {0,0,0,0};
    float targetDistanceCm = 0.0f;
    float startYaw = 0.0f;

    const float travelRPM = 150.0f;
    const float rotateRPM = 120.0f;
};
