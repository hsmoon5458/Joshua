#pragma once

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/perception/proto/perception_packet.pb.h"

namespace robot::perception {

// Common lifecycle and sampling interface for all sensors.
class PerceptionInterface {
 public:
  PerceptionInterface() = default;
  virtual ~PerceptionInterface() = default;

  virtual absl::Status Init() = 0;
  virtual std::string GetId() = 0;
  virtual absl::StatusOr<robot::perception::PerceptionPacket> GetData() = 0;
  virtual absl::Status Teardown() = 0;
};

}  // namespace robot::perception
