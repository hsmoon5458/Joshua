#pragma once

#include "robot/perception/interfaces/perception_interface.h"

namespace robot::perception {

// Common interface for sensors that publish linear or rotary position.
class PositionInterface : public PerceptionInterface {
 public:
  ~PositionInterface() override = default;
};

}  // namespace robot::perception
