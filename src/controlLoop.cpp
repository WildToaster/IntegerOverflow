#include "controlLoop.h"

namespace pid {

PIDPacket pidStep(float currentError, float currentTime, const PIDPacket& previousPacket, const PIDGains& gains) {
    float dt = currentTime - previousPacket.lastTime; // Delta (elapsed) time is used to keep things constant, even if there's a lag spike
    PIDPacket result;

    result.errorSum = previousPacket.errorSum + currentError / dt;

    // If the error has a zero crossing, reset the error sum to prevent windup
    if (currentError > 0 != previousPacket.lastError > 0) result.errorSum = 0;

    float p = gains.p * currentError;
    float i = gains.i * result.errorSum;
    float d = gains.d * (previousPacket.lastError - currentError) / dt;

    result.output = p + i + d;
    result.lastError = currentError;
    result.lastTime = currentTime;

    return result;
}

}