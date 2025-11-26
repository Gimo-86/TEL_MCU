#pragma once
#include <Arduino.h>

class SbusParser {
public:
    bool parseFrame(const uint8_t* frame, uint16_t ch[16], uint8_t &flags);
    void printFrame(const uint8_t* frame, const uint16_t ch[16], uint8_t flags);

    uint32_t framesSeen = 0;
    uint32_t malformedSeen = 0;
};