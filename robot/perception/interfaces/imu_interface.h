#pragma once

#include "robot/perception/interfaces/perception_interface.h"

namespace robot::perception {

// Common interface for inertial measurement units.
class ImuInterface : public PerceptionInterface {
 public:
  ~ImuInterface() override = default;
};

}  // namespace robot::perception
