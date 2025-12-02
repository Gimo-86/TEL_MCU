#pragma once
#include <Arduino.h>

/*--------------------------------------------------

    Hardware Configuration for Mega2560

    Available Pure Digital pins:
    22-43, 47-53

    Available PWM pins:
    |2|, |3|, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 44, 45, 46

    Available EXINT pins:
    |2|, |3|, |18|, |19|, 20, 21

    Available Communication Serial ports:
    TX: 1(0), 14(1), 16(2), |18|(3)
    RX: 0(0), 15(1), 17(2), |19|(3)
    SDA: 20
    SCL: 21

--------------------------------------------------*/

/* Configuration Parameters */

// Baud rate
static const uint32_t USB_BAUD = 115200;
static const uint32_t SBUS_BAUD = 100000;

//SBUS readout neutral value
static const uint16_t SBUS_NEUTRAL = 992;

// Deadzone around SBUS neutral to treat small joystick deviations as zero
static const uint16_t SBUS_DEADZONE = 40; // adjust as needed

// Enable motor output
static const bool ENABLE_MOTOR_OUTPUT = true;

// Motor pins 1: Front Left, 2: Front Right, 3: Rear Left, 4: Rear Right
static const uint8_t MOTOR1_PWM_PIN  = 4;
static const uint8_t MOTOR1_DIR_PIN  = 37; // 22 buggy
static const uint8_t MOTOR1_DIR1_PIN = 38; // 23 buggy
static const uint8_t FL_ENA = 2;

static const uint8_t MOTOR2_PWM_PIN  = 5;
static const uint8_t MOTOR2_DIR_PIN  = 25;
static const uint8_t MOTOR2_DIR1_PIN = 24;
static const uint8_t FR_ENA = 3;

static const uint8_t MOTOR3_PWM_PIN  = 6;
static const uint8_t MOTOR3_DIR_PIN  = 26;    
static const uint8_t MOTOR3_DIR1_PIN = 27;
static const uint8_t RL_ENA = 18;

static const uint8_t MOTOR4_PWM_PIN  = 7;
static const uint8_t MOTOR4_DIR_PIN  = 29;
static const uint8_t MOTOR4_DIR1_PIN = 28;
static const uint8_t RR_ENA = 19;


// SBUS channel to control motors
// Note: parser provides `ch[]` as zero-based indexes: CH1 -> ch[0], CH2 -> ch[1], etc.

static const uint8_t CH_ROTATE = 0;              // CH1: rotate left / right (yaw)
static const uint8_t CH_FORWARD_BACKWARD = 1;    // CH2: forward / backward (throttle)
static const uint8_t CH_STRAFE_LEFT_RIGHT = 3;   // CH4: strafe left / right


// Elevation (CH3)
// CH3 -> ch[2]
static const uint8_t CH_ELEVATE = 2;                // CH3: barrel elevation control
// Elevation control parameters
static const uint8_t  ELEVATION_R_PWM_PIN   = 8;    // First PWM pin for elevation motor driver
static const uint8_t  ELEVATION_L_PWM_PIN   = 13;   // Second PWM pin for elevation motor driver
static const uint8_t  ELEVATION_R_EN_PIN    = 52;   // Right bridge enable pin
static const uint8_t  ELEVATION_L_EN_PIN    = 53;   // Left bridge enable pin
static const uint8_t  ELEVATION_POWER_LEVEL = 250;  // default max PWM (0..255)
static const uint16_t ELEVATION_DEADZONE    = 8;    // treat within ±deadzone around NEUTRAL as stop


// Fire (CH5) - momentary switch
// CH5 -> ch[4]
static const uint8_t CH_FIRE = 4;

// Aiming (CH6)
// CH6 -> ch[5]
static const uint8_t CH_AIMING = 5;


// Encoder / Motor closed-loop parameters
static const float  ENCODER_PPR     = 950.0f;   // pulses per revolution (EXINT = CHANGE)
static const int    CONFIG_MIN_PWM  = 15;       // minimum PWM to overcome deadzone
static const float  CONFIG_MAX_RPM  = 300.0f;   // max RPM mapping for motor outputs

// PID gains (one set per motor)
static const float PID_FL_KP = 1.00;
static const float PID_FL_KI = 0.00;
static const float PID_FL_KD = 0.02;

static const float PID_FR_KP = 1.0;
static const float PID_FR_KI = 0.00;
static const float PID_FR_KD = 0.02;

static const float PID_RL_KP = 1.00;
static const float PID_RL_KI = 0.00;
static const float PID_RL_KD = 0.02;

static const float PID_RR_KP = 1.00;
static const float PID_RR_KI = 0.00;
static const float PID_RR_KD = 0.02;

// CH7 -> ch[6]
static const uint8_t    CH_SET           = 6;
// ------------------- PWM Compensation -------------------
static float kFL = 1.00;
static float kFR = 0.95;
static float kRL = 0.92;
static float kRR = 0.96;

// Per-wheel direction multiplier: set to 1 for normal, -1 to invert
// Use these to correct wiring/polarity so that control mapping (CH2/CH4/CH1)
// always behaves as expected without changing the mecanum math.
static const int MOTOR_FL_DIR = 1; // Front Left
static const int MOTOR_FR_DIR = 1; // Front Right
static const int MOTOR_RL_DIR = 1; // Rear Left
static const int MOTOR_RR_DIR = 1; // Rear Right

// Limit maximum PWM output (0..255). Set to 40% by default.
static const uint8_t MAX_MOTOR_PWM = (uint8_t)(255 * 0.40); // ~102

// ---- 車體參數 ----
static const float WHEEL_DIAMETER_CM = 12.7;
static const float WHEEL_CIRC = WHEEL_DIAMETER_CM * 3.14159;

// ---- IMU ----
static const float IMU_ALPHA = 0.98;

// Threshold: user specified 992 as baseline; when > FIRE_THRESHOLD treat as trigger
static const uint16_t FIRE_THRESHOLD = 992;

// Linear actuator wiring (adjust pins to match your hardware)
// NOTE: choose PWM-capable pin for ACTUATOR_PWM_PIN if you want speed control.
static const uint8_t ACTUATOR_PWR_R_PIN   = 9;    // Power pin for actuator motor driver
static const uint8_t ACTUATOR_EN_R        = 48;   // direction pin Retrection
static const uint8_t ACTUATOR_PWR_E_PIN   = 10;   // Power pin for actuator motor driver Extension
static const uint8_t ACTUATOR_EN_E        = 49;   // direction pin Extension


// Actuator safety/timeouts
static const uint32_t ACT_MOVE_TIMEOUT_MS = 8000; // ms timeout for a single extend/retract
static const uint8_t  ACT_POWER           = 1;    // Power ON

// Fire motors (two 775 brushed motors) — powered when AIMING is ON
static const uint8_t FIRE_MOTOR1_PWM_PIN  = 11;   // PWM pin for first firing motor
static const uint8_t FIRE_MOTOR2_PWM_PIN  = 12;   // PWM pin for second firing motor
static const uint8_t FIRE_MOTOR1_DIR_PIN1 = 33;
static const uint8_t FIRE_MOTOR1_DIR_PIN2 = 34;
static const uint8_t FIRE_MOTOR2_DIR_PIN1 = 35;
static const uint8_t FIRE_MOTOR2_DIR_PIN2 = 36;
static const uint8_t FIRE_MOTOR_POWER     = 100;   // PWM power level when enabled (0..255)
