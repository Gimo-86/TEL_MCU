#include "ActuatorControl.h"
#include "Config.h"

enum class ActState { Zeroing, Idle, Retracting, DelayBeforeExtend, Extending, Error };

static ActState state = ActState::Idle;
static uint32_t stateStartMs = 0;
static bool busy = false;
static char lastErr[64] = "";


static void actuatorDrive(int dir) {
    dir = constrain(dir, -255, 255);

    if (dir == 0) {
        digitalWrite(ACTUATOR_EN_E, LOW);
        digitalWrite(ACTUATOR_EN_R, LOW);
        digitalWrite(ACTUATOR_PWR_E_PIN, LOW);
        digitalWrite(ACTUATOR_PWR_R_PIN, LOW);
        return;
    }

    if (dir > 0) {
        // extend
        digitalWrite(ACTUATOR_EN_E, HIGH);
        digitalWrite(ACTUATOR_EN_R, HIGH);
        digitalWrite(ACTUATOR_PWR_E_PIN, 1);
        digitalWrite(ACTUATOR_PWR_R_PIN, 0);
    } else {
        // retract
        digitalWrite(ACTUATOR_EN_E, HIGH);
        digitalWrite(ACTUATOR_EN_R, HIGH);
        digitalWrite(ACTUATOR_PWR_E_PIN, 0);
        digitalWrite(ACTUATOR_PWR_R_PIN, 1);
    }
}


void actuatorStopMotor() {
    digitalWrite(ACTUATOR_EN_E, LOW);
    digitalWrite(ACTUATOR_EN_R, LOW);
    digitalWrite(ACTUATOR_PWR_E_PIN, LOW);
    digitalWrite(ACTUATOR_PWR_R_PIN, LOW);
}

void initActuator() {
    pinMode(ACTUATOR_PWR_E_PIN, OUTPUT);
    pinMode(ACTUATOR_EN_E, OUTPUT);
    pinMode(ACTUATOR_PWR_R_PIN, OUTPUT);
    pinMode(ACTUATOR_EN_R, OUTPUT);

    actuatorDrive(ACT_POWER);
    delay(3000);
    actuatorDrive(-ACT_POWER);
    delay(1500);
    actuatorStopMotor();

    state = ActState::Idle;
    busy = false;
    lastErr[0] = '\0';
}

bool startFireCycle() {
    if (busy) return false;

    busy = true;
    state = ActState::Retracting;
    stateStartMs = millis();
    lastErr[0] = '\0';

    actuatorDrive(-ACT_POWER);  // begin retracting (negative direction)
    return true;
}

bool actuatorIsBusy() { return busy; }

const char* actuatorLastError() { return lastErr; }

void updateActuator() {
    if (state == ActState::Idle || state == ActState::Error) return;

    uint32_t now = millis();

    // ----------------------------------
    // Retracting
    // ----------------------------------
    if (state == ActState::Retracting) {
        if (now - stateStartMs >= 1600) {
            actuatorStopMotor();
            state = ActState::DelayBeforeExtend;
            stateStartMs = now;
            return;
        }

        if (now - stateStartMs > ACT_MOVE_TIMEOUT_MS) {
            actuatorStopMotor();
            snprintf(lastErr, sizeof(lastErr), "Retract timeout");
            Serial.println(lastErr);
            busy = false;
            state = ActState::Error;
            return;
        }
    }

    // ----------------------------------
    // Delay before extending
    // ----------------------------------
    if (state == ActState::DelayBeforeExtend) {
        if (now - stateStartMs >= 10) {
            state = ActState::Extending;
            stateStartMs = now;
            actuatorDrive(-ACT_POWER);
        }
        return;
    }

    // ----------------------------------
    // Extending
    // ----------------------------------
    if (state == ActState::Extending) {
        if (now - stateStartMs >= 1500) {
            actuatorStopMotor();
            busy = false;
            state = ActState::Idle;
            Serial.println("Actuator: fire cycle complete");
            return;
        }

        if (now - stateStartMs > ACT_MOVE_TIMEOUT_MS) {
            actuatorStopMotor();
            snprintf(lastErr, sizeof(lastErr), "Extend timeout");
            Serial.println(lastErr);
            busy = false;
            state = ActState::Error;
            return;
        }
    }
}
