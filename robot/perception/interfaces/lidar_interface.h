#pragma once

#include "robot/perception/interfaces/perception_interface.h"

namespace robot::perception {

// Common interface for lidar sensors that publish range scans or point clouds.
class LidarInterface : public PerceptionInterface {
 public:
  ~LidarInterface() override = default;
};

}  // namespace robot::perception
