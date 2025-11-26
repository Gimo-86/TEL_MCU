#include <Arduino.h>
#include "Config.h"
#include "SbusParser.h"
#include "MotorControl.h"
#include "MecanumControl.h"
#include "ActuatorControl.h"
#include "ElevationControl.h"
#include "FireMotorControl.h"

SbusParser sbus;

// ==== 各馬達 PID 預設 ====
PIDController pidFL(PID_FL_KP, PID_FL_KI, PID_FL_KD);
PIDController pidFR(PID_FR_KP, PID_FR_KI, PID_FR_KD);
PIDController pidRL(PID_RL_KP, PID_RL_KI, PID_RL_KD);
PIDController pidRR(PID_RR_KP, PID_RR_KI, PID_RR_KD);

// ==== 各馬達物件 ====
Motor motorFL(MOTOR1_PWM_PIN, MOTOR1_DIR_PIN, MOTOR1_DIR1_PIN, 22, 23, PID_FL_KP, PID_FL_KI, PID_FL_KD); // Front Left
Motor motorFR(MOTOR2_PWM_PIN, MOTOR2_DIR_PIN, MOTOR2_DIR1_PIN, 24, 25, PID_FR_KP, PID_FR_KI, PID_FR_KD); // Front Right
Motor motorRL(MOTOR3_PWM_PIN, MOTOR3_DIR_PIN, MOTOR3_DIR1_PIN, 26, 27, PID_RL_KP, PID_RL_KI, PID_RL_KD); // Rear Left
Motor motorRR(MOTOR4_PWM_PIN, MOTOR4_DIR_PIN, MOTOR4_DIR1_PIN, 28, 29, PID_RR_KP, PID_RR_KI, PID_RR_KD); // Rear Right    

void setup() {
    Serial.begin(USB_BAUD);
    Serial1.begin(SBUS_BAUD, SERIAL_8E2);
    Serial1.setTimeout(5);

    initMecanum();  // 一定要初始化馬達
    initActuator(); // initialize actuator control and pins
    initElevation(); // initialize elevation motor
    initFireMotors(); // initialize firing motor outputs

    Serial.println("SBUS + Mecanum Ready");
}

void loop() {

    if (!Serial1.available()) return;

    if (Serial1.read() != 0x0F) return;

    uint8_t frame[25];
    frame[0] = 0x0F;

    if (Serial1.readBytes(frame + 1, 24) != 24) return;
    if (frame[24] != 0x00) return;

    uint16_t ch[16];
    uint8_t flags;

    sbus.parseFrame(frame, ch, flags);

    // Debug 可開可關
    sbus.printFrame(frame, ch, flags);

    // CH6 作為 aiming 開關 (active when > AIMING_THRESHOLD)
    static bool aimingState = true;
    bool newAiming = (ch[CH_AIMING] > AIMING_THRESHOLD);
    if (newAiming != aimingState) {
        aimingState = newAiming;
        if (aimingState) {
            Serial.println("AIM: ON");
            setFireMotors(false); // power firing motors when aiming off
        } else {
            Serial.println("AIM: OFF");
            setFireMotors(true); // cut power when aiming on
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

    // CH3 -> elevation control (continuous)
    setElevationFromChannel(ch[CH_ELEVATE]);

    // ⭐ 這裡是真的在控制馬達（CH2 + CH4）
    mecanumDrive(
        ch[CH_FORWARD_BACKWARD],     // 你的油門
        ch[CH_STRAFE_LEFT_RIGHT],    // 左右平移
        ch[CH_ROTATE]                // 旋轉由 CH1 控制
    );
}
