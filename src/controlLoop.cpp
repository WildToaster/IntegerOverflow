#include "controlLoop.h"
#include <cmath>

namespace pid {

PIDPacket pidStep(float currentError, float currentTime, const PIDPacket& previousPacket, const PIDGains& gains) {
    float dt = currentTime - previousPacket.lastTime; // Delta (elapsed) time is used to keep things constant, even if there's a lag spike
    
    if (dt == 0) return previousPacket;
    
    PIDPacket result;
    result.setpoint = previousPacket.setpoint;
    if (gains.slewRate == -1) {
      result.slew = 1;
    } else {
      result.slew = previousPacket.slew;
    }

    if (std::abs(result.errorSum) < gains.maxErrorSum)
      result.errorSum = previousPacket.errorSum + currentError / dt;
    
    if (std::abs(result.errorSum) > gains.maxErrorSum)
      result.errorSum = std::copysign(gains.maxErrorSum, result.errorSum); // Copies the sign from errorSum to the maxErrorSums

    // If the error has a zero crossing, reset the error sum to prevent windup
    if (currentError > 0 != previousPacket.lastError > 0)
      result.errorSum = 0;

    float p = gains.p * currentError;
    float i = gains.i * result.errorSum;
    float d = gains.d * (previousPacket.lastError - currentError) / dt;

    result.output = p + i + d;
    result.lastError = currentError;
    result.lastTime = currentTime;

    // Clamp output values in -100 to 100
    if (result.output > 100) result.output = 100;
    else if (result.output < -100) result.output = -100;

    // Slew
    if (result.slew < 1) {
      float slewIncrease = result.slew * gains.slewRate - result.slew;
      // printf("Slew: %f %f\n", result.slew, slewIncrease);
      if (slewIncrease > gains.maxSlewRate) slewIncrease = gains.maxSlewRate;
      result.slew += slewIncrease;
    }
    if (result.slew > 1) result.slew = 1;

    result.output *= result.slew;

    printf("PID %0.3f %0.3f %0.3f %0.3f %f\n", currentError, p, i, d, result.output);
    return result;
}

//graphing data, used for PID tuning
void graphPID(vex::brain& brain, std::vector<float> errorHistory, std::vector<float> powerHistory, int goal, float error, int time) {
  //goal is the PID goal (driveDistance)
  //error history is a list of all of the errors (range is 0 to driveDistance)
  //powerHistory is a list of the power applied (range is -1 to 1)
  //error is the current error
  //time is the current time, in milliseconds
  
  //Setup: clear screen and draw the target line
  brain.Screen.clearScreen();
  brain.Screen.setPenWidth(2);
  brain.Screen.setPenColor(vex::white);
  brain.Screen.drawLine(0, 60, 480, 60);
  brain.Screen.setPenWidth(1);
  brain.Screen.setPenColor(vex::green);

  //display final error and time
  brain.Screen.setCursor(1, 1);
  brain.Screen.clearLine(1);
  brain.Screen.print(" Final Error: ");
  brain.Screen.print(error);
  brain.Screen.print("    Time: ");
  brain.Screen.print(time);
  
  //define the borders of the graph
  int minY = 60; //error = 0 (robot is at target)
  int maxY = 230; //error = driveDistance (Robot is at start)
  int minX = 10; //time = beginning
  int maxX = 470; //time = end
  
  //loop through each data point and graph it
  for (int i = 0; i < errorHistory.size() - 1; i++) { 
    int x = minX + (maxX - minX) * i / errorHistory.size(); //find the x-value of this data point
    
    //graph velocity
    brain.Screen.setPenColor(vex::green);
    brain.Screen.drawLine(x, minY + (float)errorHistory.at(i) / goal * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), minY + (float)errorHistory.at(i + 1) / goal * (maxY - minY));
    
    //graph power, changing color based on direction
    if (powerHistory.at(i) > 0) {
      brain.Screen.setPenColor(vex::orange);
    } else {
      brain.Screen.setPenColor(vex::yellow);
    }
    
    brain.Screen.drawLine(x, maxY - std::abs(powerHistory.at(i)) * (maxY - minY), x + (float)(maxX - minX) / errorHistory.size(), maxY - std::abs(powerHistory.at(i + 1)) * (maxY - minY));
  }

  printf("done graphing\n");
}

}