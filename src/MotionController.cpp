#include "MotionController.h"
#include "Config.h"
#include "MotorControl.h"
#include <Arduino.h>
#include "MecanumControl.h"

AutoDrive::AutoDrive(MotorSystem* m, IMU* i)
  : mec(m), imu(i) {
    motorInstances[0] = &m->FL;
    motorInstances[1] = &m->FR;
    motorInstances[2] = &m->RL;
    motorInstances[3] = &m->RR;
  }

void AutoDrive::start() {
    if (running) return;
    running = true;
    Serial2.println("AutoDrive started");
    phase = 0;
    phaseStart = millis();
    // record starting encoder counts
    for (int i = 0; i < 4; ++i) {
        if (motorInstances[i]) startCounts[i] = motorInstances[i]->getPulseCount();
        else startCounts[i] = 0;
    }
}

void AutoDrive::stop() {
    if (!running) return;
    running = false;
    phase = 0;
    mec->stop();
    Serial2.println("AutoDrive stopped");
}

void AutoDrive::update() {
    if (!running) return;

    switch (phase) {
        case 0: {
            // Begin forward 300 cm
            targetDistanceCm = 300.0f;
            float m1 = travelRPM * kFL;
            float m2 = travelRPM * kFR;
            float m3 = travelRPM * kRL;
            float m4 = travelRPM * kRR;
            mec->setTargetRPMs(m1, m2, m3, m4);
            phase = 1;
            break;
        }

        case 1: {
            // Check distance traveled using encoders (use compensation factors)
            long counts[4];
            for (int i = 0; i < 4; ++i) counts[i] = motorInstances[i] ? motorInstances[i]->getPulseCount() : 0;

            float d1 = (counts[0] - startCounts[0]) / ENCODER_PPR * WHEEL_CIRC * kFL;
            float d2 = (counts[1] - startCounts[1]) / ENCODER_PPR * WHEEL_CIRC * kFR;
            float d3 = (counts[2] - startCounts[2]) / ENCODER_PPR * WHEEL_CIRC * kRL;
            float d4 = (counts[3] - startCounts[3]) / ENCODER_PPR * WHEEL_CIRC * kRR;

            float avg = (fabs(d1) + fabs(d2) + fabs(d3) + fabs(d4)) / 4.0f;
            if (avg >= targetDistanceCm) {
                mec->stop();
                // prepare for rotation
                phase = 2;
                phaseStart = millis();
            }
            break;
        }

        case 2: {
            // Start left rotation ~90 deg
            startYaw = imu->getYaw();
            float m1 = -rotateRPM * kFL;
            float m2 =  rotateRPM * kFR;
            float m3 = -rotateRPM * kRL;
            float m4 =  rotateRPM * kRR;
            mec->setTargetRPMs(m1, m2, m3, m4);
            phase = 3;
            break;
        }

        case 3: {
            // Monitor yaw change (use IMU)
            float yaw = imu->getYaw();
            float dy = yaw - startYaw;
            // normalize to [-180,180]
            while (dy > 180.0f) dy -= 360.0f;
            while (dy < -180.0f) dy += 360.0f;

            if (fabs(dy) >= 85.0f) {
                mec->stop();
                // prepare to move forward 125cm
                for (int i = 0; i < 4; ++i) startCounts[i] = motorInstances[i] ? motorInstances[i]->getPulseCount() : 0;
                targetDistanceCm = 125.0f;
                phase = 4;
            }
            break;
        }

        case 4: {
            // Move forward targetDistanceCm
            long counts[4];
            for (int i = 0; i < 4; ++i) counts[i] = motorInstances[i] ? motorInstances[i]->getPulseCount() : 0;

            float d1 = (counts[0] - startCounts[0]) / ENCODER_PPR * WHEEL_CIRC * kFL;
            float d2 = (counts[1] - startCounts[1]) / ENCODER_PPR * WHEEL_CIRC * kFR;
            float d3 = (counts[2] - startCounts[2]) / ENCODER_PPR * WHEEL_CIRC * kRL;
            float d4 = (counts[3] - startCounts[3]) / ENCODER_PPR * WHEEL_CIRC * kRR;

            float avg = (fabs(d1) + fabs(d2) + fabs(d3) + fabs(d4)) / 4.0f;
            if (avg >= targetDistanceCm) {
                mec->stop();
                running = false;
                phase = 0;
            } else {
                float m1 = travelRPM * kFL;
                float m2 = travelRPM * kFR;
                float m3 = travelRPM * kRL;
                float m4 = travelRPM * kRR;
                mec->setTargetRPMs(m1, m2, m3, m4);
            }
            break;
        }

        default:
            mec->stop();
            running = false;
            phase = 0;
            break;
    }
}
