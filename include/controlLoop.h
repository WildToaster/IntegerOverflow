#pragma once

namespace pid {

struct PIDPacket {
    float output = 0;
    float lastError = 0;
    float lastTime = 0;
    float errorSum = 0;
};

struct PIDGains {
    double p = 0;
    double i = 0;
    double d = 0;
};

extern PIDPacket pidStep(float currentError, float currentTime, const PIDPacket& previousPacket, const PIDGains& gains);

}