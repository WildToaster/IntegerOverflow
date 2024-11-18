#pragma once

#include <vector>
#include "vex.h"

namespace pid {

struct PIDPacket {
    float setpoint = 0;
    float output = 0;
    float lastError = 0;
    float lastTime = 0;
    float errorSum = 0;
    float slew = 0.01;
};

struct PIDGains {
    double p = 0;
    double i = 0;
    double d = 0;
    double antiWindup = 0;
    double slewRate = 0;

    PIDGains(double p, double i, double d, double antiWindup, double slewRate): p(p), i(i), d(d), antiWindup(antiWindup), slewRate(slewRate) {}
};

// The output will always be between -100 and 100.
extern PIDPacket pidStep(float currentError, float currentTime, const PIDPacket& previousPacket, const PIDGains& gains);

extern void graphPID(vex::brain& brain, std::vector<float> errorHistory, std::vector<float> powerHistory, int goal, float finalError, int finalTime);

}