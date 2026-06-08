#include <iostream>
#include <vector>

#include "RobotController.h"

int main() {
  RobotController controller;

  std::vector<SensorData> samples;
  for (unsigned long t = 0; t <= 5000; t += 100) {
    SensorData data;
    data.timeMs = t;
    data.distanceCm = 80.0f;
    data.rollDeg = 0.0f;
    data.color[0] = 820;
    data.color[1] = 300;
    data.color[2] = 310;
    data.color[3] = 830;
    if (t > 3600) {
      data.color[0] = 310;
      data.color[1] = 320;
      data.color[2] = 330;
      data.color[3] = 315;
    }
    samples.push_back(data);
  }

  for (const SensorData& data : samples) {
    MotorCommand cmd = controller.update(data);
    std::cout << data.timeMs << " ms"
              << " state=" << controller.stateName()
              << " FL=" << cmd.frontLeft
              << " FR=" << cmd.frontRight
              << " RL=" << cmd.rearLeft
              << " RR=" << cmd.rearRight
              << " servo=" << cmd.servoDeg
              << " stop=" << cmd.stop << '\n';
  }

  return 0;
}
