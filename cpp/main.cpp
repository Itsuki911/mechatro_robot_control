#include <iostream>
#include <vector>

#include "RobotController.h"

int main() {
  RobotController controller;
  std::vector<SensorData> samples;

  for (unsigned long t = 0; t <= 6000; t += 100) {
    SensorData data;
    data.timeMs = t;
    data.distanceCm = 80.0f;
    data.rollDeg = 0.0f;
    data.color[0] = 260;
    data.color[1] = 780;
    data.color[2] = 790;
    data.color[3] = 270;
    if (t > 4200) {
      data.color[0] = 260;
      data.color[1] = 300;
      data.color[2] = 760;
      data.color[3] = 830;
    }
    samples.push_back(data);
  }

  for (const SensorData& data : samples) {
    MotorCommand cmd = controller.update(data);
    std::cout << data.timeMs << " ms"
              << " state=" << controller.stateName()
              << " drive=" << cmd.driveSpeed
              << " servo=" << cmd.servoDeg
              << " stop=" << cmd.stop << '\n';
  }
  return 0;
}
