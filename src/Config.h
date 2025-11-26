#pragma once
#include <Arduino.h>

// Baud rate
static const uint32_t USB_BAUD = 115200;
static const uint32_t SBUS_BAUD = 100000;

// Enable motor output?
static const bool ENABLE_MOTOR_OUTPUT = true;

// Motor pins 1: Front Left, 2: Front Right, 3: Rear Left, 4: Rear Right
static const uint8_t MOTOR1_PWM_PIN  = 2;
static const uint8_t MOTOR1_DIR_PIN  = 3;
static const uint8_t MOTOR1_DIR1_PIN = 4;
static const uint8_t FL_ENA = 42;
static const uint8_t FL_ENB = 39;

static const uint8_t MOTOR2_PWM_PIN  = 5;
static const uint8_t MOTOR2_DIR_PIN  = 6;
static const uint8_t MOTOR2_DIR1_PIN = 7;
static const uint8_t FR_ENA = 40;
static const uint8_t FR_ENB = 38;

static const uint8_t MOTOR3_PWM_PIN  = 9;
static const uint8_t MOTOR3_DIR_PIN  = 31;
static const uint8_t MOTOR3_DIR1_PIN = 10;
static const uint8_t RL_ENA = 41;
static const uint8_t RL_ENB = 37;

static const uint8_t MOTOR4_PWM_PIN  = 11;
static const uint8_t MOTOR4_DIR_PIN  = 13;
static const uint8_t MOTOR4_DIR1_PIN = 12;
static const uint8_t RR_ENA = 38;
static const uint8_t RR_ENB = 36;

// SBUS channel to control motors
// Note: parser provides `ch[]` as zero-based indexes: CH1 -> ch[0], CH2 -> ch[1], etc.
static const uint8_t CH_ROTATE = 0;               // CH1: rotate left / right (yaw)
static const uint8_t CH_FORWARD_BACKWARD = 1;     // CH2: forward / backward (throttle)
static const uint8_t CH_STRAFE_LEFT_RIGHT = 3;    // CH4: strafe left / right

// Elevation (CH3)
// CH3 -> ch[2]
static const uint8_t CH_ELEVATE = 2;             // CH3: barrel elevation control
// Elevation control parameters
static const uint8_t ELEVATION_PWM_PIN = 8;      // PWM pin for elevation motor driver (change to your wiring)
static const uint8_t ELEVATION_DIR_PIN = 41;     // DIR pin for elevation motor driver (HIGH=increase angle)
static const uint8_t ELEVATION_POWER_LEVEL = 255;// default max PWM (0..255)
static const uint16_t ELEVATION_DEADZONE = 8;    // treat within ±deadzone around NEUTRAL as stop
static const uint16_t SBUS_NEUTRAL = 992;        // neutral value for SBUS channels

// You can add inversion flags or deadzone thresholds here if needed
// Aiming (CH6)
// CH6 -> ch[5]
static const uint8_t CH_AIMING = 5;               // CH6: aiming switch
// Threshold to decide ON/OFF for a switch channel (SBUS values range ~192..1792)
static const uint16_t AIMING_THRESHOLD = 992;
// Fire (CH5) - momentary switch
// CH5 -> ch[4]

// Encoder / Motor closed-loop parameters
static const float ENCODER_PPR = 950.0;          // pulses per revolution (調整為你的編碼器)
static const int CONFIG_MIN_PWM = 15;            // minimum PWM to overcome deadzone
static const float CONFIG_MAX_RPM = 120.0;      // max RPM mapping for motor outputs

// PID gains (one set per motor)
static const float PID_FL_KP = 0.38;
static const float PID_FL_KI = 0.0065;
static const float PID_FL_KD = 0.038;

static const float PID_FR_KP = 0.34;
static const float PID_FR_KI = 0.0058;
static const float PID_FR_KD = 0.037;

static const float PID_RL_KP = 0.33;
static const float PID_RL_KI = 0.006;
static const float PID_RL_KD = 0.04;

static const float PID_RR_KP = 0.30;
static const float PID_RR_KI = 0.006;
static const float PID_RR_KD = 0.038;

// SBUS channel to set global RPM override (optional)
// CH7 -> ch[6]
static const uint8_t CH_RPM_SET = 6;
static const uint16_t RPM_SET_DEADZONE = 20; // small deadzone around neutral
static const uint8_t CH_FIRE = 4;                 // CH5: momentary fire switch
// Threshold: user specified 992 as baseline; when > FIRE_THRESHOLD treat as trigger
static const uint16_t FIRE_THRESHOLD = 992;

// Linear actuator wiring (adjust pins to match your hardware)
// NOTE: choose PWM-capable pin for ACTUATOR_PWM_PIN if you want speed control.
static const uint8_t ACTUATOR_PWR_R_PIN     = 44;   // Power pin for actuator motor driver
static const uint8_t ACTUATOR_EN_R          = 43;   // direction pin Retrection
static const uint8_t ACTUATOR_PWR_E_PIN     = 39;   // Power pin for actuator motor driver Extension
static const uint8_t ACTUATOR_EN_E          = 38;   // direction pin Extension


// Limit switches (wired so that ACTIVE == LOW). Adjust if your switches are active HIGH.
static const uint8_t ACT_LIMIT_EXT_PIN    = 22;   // limit switch for fully extended
static const uint8_t ACT_LIMIT_RET_PIN    = 23;   // limit switch for fully retracted
static const bool ACT_LIMIT_ACTIVE_LOW    = true; // true if switch reads LOW when triggered

// Actuator safety/timeouts
static const uint32_t ACT_MOVE_TIMEOUT_MS = 8000; // ms timeout for a single extend/retract
static const uint8_t ACT_POWER            = 1;    // Power ON

// Fire motors (two 775 brushed motors) — powered when AIMING is ON
static const uint8_t FIRE_MOTOR1_PWM_PIN = 45;   // PWM pin for first firing motor
static const uint8_t FIRE_MOTOR2_PWM_PIN = 46;   // PWM pin for second firing motor
static const uint8_t FIRE_MOTOR1_DIR_PIN1 = 30;
static const uint8_t FIRE_MOTOR1_DIR_PIN2 = 35;
static const uint8_t FIRE_MOTOR2_DIR_PIN1 = 33;
static const uint8_t FIRE_MOTOR2_DIR_PIN2 = 42;
static const uint8_t FIRE_MOTOR_POWER = 20;     // PWM power level when enabled (0..255)
