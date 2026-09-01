#pragma once

#include "robot/perception/interfaces/perception_interface.h"

namespace robot::perception {

// Common interface for radar sensors.
class RadarInterface : public PerceptionInterface {
 public:
  ~RadarInterface() override = default;
};

}  // namespace robot::perception
