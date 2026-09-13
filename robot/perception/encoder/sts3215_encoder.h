#pragma once

#include <any>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/comm/interfaces/message_transport.h"
#include "robot/perception/interfaces/encoder_interface.h"
#include "robot/perception/proto/perception.pb.h"
#include "robot/perception/proto/perception_packet.pb.h"

namespace robot::perception {

class Sts3215Encoder : public EncoderInterface {
 public:
  explicit Sts3215Encoder(const std::shared_ptr<robot::comm::MessageTransport>& transport,
                          const robot::perception::Encoder& encoder_config);
  ~Sts3215Encoder() override = default;

  absl::Status Init() override;
  std::string GetId() override;
  absl::StatusOr<robot::perception::PerceptionPacket> GetData() override;
  absl::StatusOr<float> GetPosition() override;
  absl::Status Teardown() override;

 private:
  std::shared_ptr<robot::comm::MessageTransport> transport_;
  std::string id_;
  uint8_t servo_id_;
  mutable robot::perception::PerceptionPacket reusable_packet_;  // Pre-allocated packet for reuse

  uint8_t calculate_checksum(std::vector<uint8_t>::const_iterator begin,
                             std::vector<uint8_t>::const_iterator end);
  std::vector<uint8_t> create_read_position_packet();
  absl::StatusOr<uint16_t> read_servo_position();
};

}  // namespace robot::perception
