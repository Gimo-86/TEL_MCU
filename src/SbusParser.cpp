#include "SbusParser.h"
#include "MotorControl.h"
#include "Config.h"

bool SbusParser::parseFrame(const uint8_t* b, uint16_t ch[16], uint8_t &flagsOut) {
    // Basic sanity checks for SBUS frame structure
    if (b == nullptr) {
        malformedSeen++;
        return false;
    }

    // Expect start byte 0x0F and end byte 0x00 at position 24
    if (b[0] != 0x0F || b[24] != 0x00) {
        malformedSeen++;
        return false;
    }

    uint8_t flags = b[23];
    // Flags normally use lower bits: bit0=ch17, bit1=ch18, bit2=frameLost, bit3=failsafe
    // If high bits are set something is suspicious — treat as malformed
    if (flags & 0xF0) {
        malformedSeen++;
        return false;
    }

    ch[0]  = (uint16_t)(((b[1]    | (b[2]  << 8))) & 0x07FF);
    ch[1]  = (uint16_t)(((b[2] >> 3 | (b[3]  << 5))) & 0x07FF);
    ch[2]  = (uint16_t)(((b[3] >> 6 | (b[4]  << 2) | (b[5] << 10))) & 0x07FF);
    ch[3]  = (uint16_t)(((b[5] >> 1 | (b[6]  << 7))) & 0x07FF);
    ch[4]  = (uint16_t)(((b[6] >> 4 | (b[7]  << 4))) & 0x07FF);
    ch[5]  = (uint16_t)(((b[7] >> 7 | (b[8]  << 1) | (b[9] << 9))) & 0x07FF);
    ch[6]  = (uint16_t)(((b[9] >> 2 | (b[10] << 6))) & 0x07FF);
    ch[7]  = (uint16_t)(((b[10]>> 5 | (b[11] << 3))) & 0x07FF);
    ch[8]  = (uint16_t)(((b[12]    | (b[13] << 8))) & 0x07FF);
    ch[9]  = (uint16_t)(((b[13]>> 3 | (b[14] << 5))) & 0x07FF);
    ch[10] = (uint16_t)(((b[14]>> 6 | (b[15] << 2) | (b[16] <<10))) & 0x07FF);
    ch[11] = (uint16_t)(((b[16]>> 1 | (b[17] << 7))) & 0x07FF);
    ch[12] = (uint16_t)(((b[17]>> 4 | (b[18] << 4))) & 0x07FF);
    ch[13] = (uint16_t)(((b[18]>> 7 | (b[19] << 1) | (b[20] << 9))) & 0x07FF);
    ch[14] = (uint16_t)(((b[20]>> 2 | (b[21] << 6))) & 0x07FF);
    ch[15] = (uint16_t)(((b[21]>> 5 | (b[22] << 3))) & 0x07FF);

    flagsOut = flags;
    framesSeen++;
    return true;
}

void SbusParser::printFrame(const uint8_t* frame, const uint16_t ch[16], uint8_t flags) {

    Serial.print("SBUS frame #"); 
    Serial.print(framesSeen);
    Serial.print(" | ch:");

    for (int i = 0; i < 16; i++) {
        Serial.print(' ');
        Serial.print(i+1);
        Serial.print(':');
        Serial.print(ch[i]);
    }

    bool frameLost = flags & 0x04;
    bool failsafe = flags & 0x08;
    bool ch17 = flags & 0x01;
    bool ch18 = flags & 0x02;

    Serial.print(" | f:");
    if (frameLost) Serial.print("LOST ");
    if (failsafe) Serial.print("FAIL ");
    if (ch17) Serial.print("CH17 ");
    if (ch18) Serial.print("CH18 ");
    Serial.println();

}
