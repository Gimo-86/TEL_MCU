#include "IMU.h"
#include <Wire.h>
#include "Config.h"
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

bool IMU::begin() {
    Wire.begin();
    // Try to contact MPU6050 at 0x68
    Wire.beginTransmission(0x68);
    Wire.write(0x6B); // PWR_MGMT_1
    Wire.write(0);    // set to zero (wake up)
    int rc = Wire.endTransmission();
    if (rc != 0) {
        // non-zero means no ACK / device not present
        return false;
    }
    delay(100);
    lastTime = micros();
    return true;
}

void IMU::readMPU() {
    Wire.beginTransmission(0x68);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(0x68, 14, true);

    ax = (Wire.read() << 8) | Wire.read();
    ay = (Wire.read() << 8) | Wire.read();
    az = (Wire.read() << 8) | Wire.read();
    gx = (Wire.read() << 8) | Wire.read();
    gy = (Wire.read() << 8) | Wire.read();
    gz = (Wire.read() << 8) | Wire.read();
}

void IMU::computeAngles(float dt) {
    // Convert raw accel to float for angle calculation
    float ax_f = (float)ax;
    float ay_f = (float)ay;
    float az_f = (float)az;

    // Compute accelerometer angles (degrees)
    float accPitch = atan2(-ax_f, sqrt(ay_f*ay_f + az_f*az_f)) * 180.0f / PI;
    float accRoll  = atan2(ay_f, az_f) * 180.0f / PI;

    // Cascaded 1st-order low-pass (2nd-order overall) for accelerometer angles
    // cutoff frequency (Hz)
    const float accCutoffHz = 5.0f; // tune as needed
    if (dt <= 0) dt = 1e-3;
    float rc = 1.0f / (2.0f * PI * accCutoffHz);
    float alpha = dt / (rc + dt);

    // first stage
    accPitch_f1 += alpha * (accPitch - accPitch_f1);
    accRoll_f1  += alpha * (accRoll  - accRoll_f1);
    // second stage
    accPitch_f2 += alpha * (accPitch_f1 - accPitch_f2);
    accRoll_f2  += alpha * (accRoll_f1  - accRoll_f2);

    // Convert raw gyro readings to degrees/sec (assume ±250 dps => 131 LSB/deg/s)
    const float gyroScale = 131.0f;
    float gx_dps = gx / gyroScale;
    float gy_dps = gy / gyroScale;
    float gz_dps = gz / gyroScale;

    // Low-pass filter the gz (yaw rate) to reduce high-frequency noise before integration
    // Use first-order RC filter: alpha = dt / (RC + dt), RC = 1/(2*pi*fc)
    float rc_gz = 1.0f / (2.0f * PI * gyroZCutoffHz);
    float alpha_gz = dt / (rc_gz + dt);
    gz_f += alpha_gz * (gz_dps - gz_f);

    // Complementary filter: trust gyro short-term, accel long-term
    pitch = IMU_ALPHA * (pitch + gy_dps * dt) + (1.0f - IMU_ALPHA) * accPitch_f2;
    roll  = IMU_ALPHA * (roll  + gx_dps * dt) + (1.0f - IMU_ALPHA) * accRoll_f2;

    // Integrate the filtered gz to obtain yaw (reduces high-frequency jitter)
    yaw += gz_f * dt;

}

void IMU::update() {
    readMPU();
    unsigned long now = micros();
    float dt = (now - lastTime) * 1e-6;
    lastTime = now;
    computeAngles(dt);
}

float IMU::getPitch() { return pitch; }
float IMU::getRoll()  { return roll; }
float IMU::getYaw()   { return yaw; }
