#pragma once

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/board/interfaces/board_channel.h"
#include "robot/perception/interfaces/position_interface.h"
#include "robot/perception/proto/perception.pb.h"
#include "robot/perception/proto/perception_packet.pb.h"

namespace robot::perception {

// Reads linear or rotary position from a compatible board channel without
// depending on its signal or communication mechanism.
class PositionSensor : public PositionInterface {
 public:
  PositionSensor(std::shared_ptr<robot::board::BoardChannel> channel,
                 const robot::perception::Sensor& sensor_config);
  ~PositionSensor() override = default;

  absl::Status Init() override;
  std::string GetId() override;
  absl::StatusOr<robot::perception::PerceptionPacket> GetData() override;
  absl::Status Teardown() override;

 private:
  std::shared_ptr<robot::board::BoardChannel> channel_;
  std::string id_;
  robot::perception::PerceptionPacket reusable_packet_;
};

}  // namespace robot::perception
