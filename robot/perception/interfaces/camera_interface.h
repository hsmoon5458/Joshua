#pragma once

#include "robot/perception/interfaces/perception_interface.h"

namespace robot::perception {

// Common interface for sensors that publish images.
class CameraInterface : public PerceptionInterface {
 public:
  ~CameraInterface() override = default;
};

}  // namespace robot::perception
