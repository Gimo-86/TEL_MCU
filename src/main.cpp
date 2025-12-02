#include <Arduino.h>
#include "Config.h"
#include "SbusParser.h"
#include "MotorControl.h"
#include "MecanumControl.h"
#include "IMU.h"
#include "MotionController.h"
#include "ActuatorControl.h"
#include "ElevationControl.h"
#include "FireMotorControl.h"

SbusParser sbus;

// ==== 馬達系統物件 ====
MotorSystem motorSystem;  
IMU imu;
AutoDrive autodrive(&motorSystem, &imu);

void setup() {
    Serial.begin(USB_BAUD);
    Serial2.begin(SBUS_BAUD, SERIAL_8E2);
    Serial2.setTimeout(5);

    motorSystem.initMecanum();  // initialize mecanum motor control

    // Initialize IMU once and check result
    if (!imu.begin()) {
        Serial.println("IMU init FAILED!");
        while (1);
    }
    Serial.println("IMU ready!");

    initActuator();             // initialize actuator control and pins
    initElevation();            // initialize elevation motor
    initFireMotors();           // initialize firing motor outputs

    Serial.println("System Ready.");
    Serial.println("SBUS + Mecanum Ready");
}

void loop() {

    // Read SBUS frame robustly: find start 0x0F, then ensure 24 bytes follow.
    uint8_t frame[25];
    const unsigned long startTimeoutMs = 50; // time to wait for start byte
    unsigned long t0 = millis();
    bool foundStart = false;
    while (millis() - t0 < startTimeoutMs) {
        if (Serial2.available()) {
            int b = Serial2.read();
            if (b == 0x0F) {
                frame[0] = 0x0F;
                foundStart = true;
                break;
            }
        }
    }
    if (!foundStart) return;

    // Wait briefly for the remaining 24 bytes to arrive
    const unsigned long waitRemainingMs = 10;
    t0 = millis();
    while (Serial2.available() < 24 && (millis() - t0) < waitRemainingMs) {
        ; // busy-wait up to timeout
    }
    if (Serial2.available() < 24) return;

    if (Serial2.readBytes(frame + 1, 24) != 24) return;
    if (frame[24] != 0x00) {
        // bad end byte — likely misaligned or noise
        //Serial.println("SBUS frame bad end byte");
        return;
    }

    uint16_t ch[16];
    uint8_t flags;

    // Debug: show raw 25-byte SBUS frame in hex to verify alignment
    //Serial.print("RAW SBUS:");
    //for (int i = 0; i < 25; i++) {
       // Serial.print(' ');
        //if (frame[i] < 16) Serial.print('0');
        //Serial.print(frame[i], HEX);
        //}
    //Serial.println();



    bool ok = sbus.parseFrame(frame, ch, flags);
    if (!ok) {
        Serial.println("SBUS parse FAILED");
    }

    // Debug 可開可關
    sbus.printFrame(frame, ch, flags);

    // CH6 作為 aiming 開關 (active when > SBUS_NEUTRAL)
    static bool aimingState = true;
    bool newAiming = (ch[CH_AIMING] > SBUS_NEUTRAL);
    if (newAiming != aimingState) {
        aimingState = newAiming;
        if (aimingState) {
            Serial.println("AIM: ON");
            setFireMotors(false);   // Stop firing motors when aiming on
        } else {
            Serial.println("AIM: OFF");
            setFireMotors(true);    // Start firing motors when aiming off
        }
    }

    // CH5 (momentary) -> fire (start one retract->extend cycle)
    static bool prevFire = false;
    // Only consider CH5 active when its value is >= FIRE_THRESHOLD; below threshold does nothing
    bool fireNow = (ch[CH_FIRE] <= FIRE_THRESHOLD);
    if (fireNow && !prevFire) {
        if (aimingState) {
            // When aiming is active, CH5 should be ignored
            Serial.println("FIRE: disabled (aiming active)");
        } else {
            if (startFireCycle()) Serial.println("FIRE: started");
            else Serial.println("FIRE: busy");
        }
    }
    prevFire = fireNow;


    // Update actuator state machine every loop
    updateActuator();

    // Update IMU
    imu.update();

    // CH7 (momentary) -> trigger AutoDrive only on rising edge (>SBUS_NEUTRAL)
    // and treat CH7==192 as emergency stop (immediate pause and clear trigger).
    static bool prevSet = false;
    static bool prevSetInitialized = false;
    bool setNow = (ch[CH_SET] > SBUS_NEUTRAL);

    // Initialize previous state on first valid frame to avoid auto-trigger at power-on
    if (!prevSetInitialized) {
        prevSet = setNow;
        prevSetInitialized = true;
    } else {
        // Emergency stop: CH7 == 192 (low command) immediately stops AutoDrive
        if (ch[CH_SET] == 192) {
            if (autodrive.isRunning()) {
                autodrive.stop();
                Serial.println("AutoDrive: EMERGENCY STOP (CH7==192)");
            }
            // Clear previous set so next rising edge will re-trigger
            prevSet = false;
        } else {
            // Normal rising-edge trigger
            if (setNow && !prevSet) {
                if (!autodrive.isRunning()) {
                    autodrive.start();
                    Serial.println("AutoDrive: START");
                }
            }
            prevSet = setNow;
        }
    }



    // CH3 -> elevation control (continuous)
    setElevationFromChannel(ch[CH_ELEVATE]);


    // If AutoDrive active, let it control motors; otherwise manual SBUS control
    if (autodrive.isRunning()) {
        autodrive.update();
    } else {
        // ⭐ 實際控制馬達（CH2 + CH4）
        motorSystem.mecanumDrive(
            ch[CH_FORWARD_BACKWARD],     // 你的油門
            ch[CH_STRAFE_LEFT_RIGHT],    // 左右平移
            ch[CH_ROTATE]                // 旋轉由 CH1 控制
        );
    }

}
