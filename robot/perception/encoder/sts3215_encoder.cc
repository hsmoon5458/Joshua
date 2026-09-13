#include "robot/perception/encoder/sts3215_encoder.h"

#include <glog/logging.h>

#include <chrono>

#include "utils/status_macros.h"

namespace robot::perception {

namespace {
constexpr auto kReadAttempt = 50;
}

Sts3215Encoder::Sts3215Encoder(const std::shared_ptr<robot::comm::MessageTransport>& transport,
                               const robot::perception::Encoder& encoder_config)
    : transport_(transport) {
  servo_id_ = encoder_config.sts3215_encoder_config().servo_id();
  id_ = GetId();
}

absl::Status Sts3215Encoder::Init() {
  ABSL_RETURN_IF_ERROR(transport_->Open());
  return absl::OkStatus();
}

absl::Status Sts3215Encoder::Teardown() {
  return absl::OkStatus();
}

std::string Sts3215Encoder::GetId() {
  auto id = "sts3215_encoder_" + std::to_string(servo_id_);
  return id;
}

absl::StatusOr<robot::perception::PerceptionPacket> Sts3215Encoder::GetData() {
  ABSL_ASSIGN_OR_RETURN(auto position, read_servo_position());
  reusable_packet_.Clear();
  reusable_packet_.set_perception_id(id_);
  reusable_packet_.set_timestamp_ns(std::chrono::duration_cast<std::chrono::nanoseconds>(
                                        std::chrono::steady_clock::now().time_since_epoch())
                                        .count());
  reusable_packet_.mutable_position()->set_position(static_cast<float>(position));

  return reusable_packet_;
}

absl::StatusOr<float> Sts3215Encoder::GetPosition() {
  ABSL_ASSIGN_OR_RETURN(auto position, read_servo_position());
  return static_cast<float>(position);
}

uint8_t Sts3215Encoder::calculate_checksum(std::vector<uint8_t>::const_iterator begin,
                                           std::vector<uint8_t>::const_iterator end) {
  uint8_t checksum = 0;
  for (auto it = begin; it != end; ++it) {
    checksum += *it;
  }
  return ~checksum & 0xFF;
}

std::vector<uint8_t> Sts3215Encoder::create_read_position_packet() {
  std::vector<uint8_t> packet = {
      static_cast<uint8_t>(0xFF),  // Start Byte 1
      static_cast<uint8_t>(0xFF),  // Start Byte 2
      servo_id_,
      static_cast<uint8_t>(0x04),  // Length (4 bytes follow: Instr, Addr, DataLength)
      static_cast<uint8_t>(0x02),  // Instruction: READ_DATA
      static_cast<uint8_t>(0x38),  // Parameter 1: Starting Address (PRESENT_POSITION_L = 56)
      static_cast<uint8_t>(0x02)   // Parameter 2: Length of Data to Read (2 bytes for position)
  };
  packet.push_back(calculate_checksum(packet.begin() + 2, packet.end()));
  return packet;
}

absl::StatusOr<uint16_t> Sts3215Encoder::read_servo_position() {
  // Use atomic Query to prevent bus contention with the driver
  std::vector<uint8_t> packet = create_read_position_packet();

  // Header (2) + ID (1) + Length (1) + Error (1) + Param1 (1) + Param2 (1) + Checksum (1) = 8 bytes
  // But wait, the synchronization loop logic in current implementation is complex.
  // The Query method in Serial does a clean read of N bytes.
  // However, the original code had a retry loop for synchronization (finding 0xFF 0xFF).
  // If we trust the Flush() in Query(), we can try a direct read.
  // If the bus is noisy, we might still need the sync logic, but that logic relies on byte-by-byte
  // reading which is hard to make atomic without a custom callback or a very specific Serial
  // method.

  // Read the fixed-size status packet in one atomic exchange.
  // If the servo responds cleanly after the flush/write, it should work.
  // If it's out of sync, we might fail. But locking the bus prevents the MAIN cause of sync loss
  // (interleaved writes).

  ABSL_ASSIGN_OR_RETURN(auto response, transport_->SendAndReceive(packet, 8));

  // 3. Validate response
  if (response.size() != 8) {
    return absl::Status(absl::StatusCode::kInternal, "Incomplete response");
  }

  // Verify Header (0xFF 0xFF)
  if (response[0] != 0xFF || response[1] != 0xFF) {
    // If we miss the header, it's a sync issue.
    // With the lock, this should be rarer.
    return absl::Status(absl::StatusCode::kInternal, "Invalid header in response");
  }

  if (response[4] != 0) {  // Error byte check
    std::string error_msg = "Servo " + std::to_string(static_cast<int>(servo_id_)) +
                            " returned an error: " + std::to_string(static_cast<int>(response[4]));
    LOG(ERROR) << error_msg;
    return absl::Status(absl::StatusCode::kInternal, error_msg);
  }

  // Validate checksum
  uint8_t calculated_checksum = calculate_checksum(response.begin() + 2, response.begin() + 7);
  uint8_t received_checksum = response[7];
  if (calculated_checksum != received_checksum) {
    std::string error_msg =
        "Checksum mismatch for servo " + std::to_string(static_cast<int>(servo_id_));
    LOG(ERROR) << error_msg;
    reusable_packet_.Clear();
    return absl::Status(absl::StatusCode::kInternal, error_msg);
  }

  // 4. Extract position bytes safely
  uint16_t position_low_byte = static_cast<uint16_t>(response[5]);
  uint16_t position_high_byte = static_cast<uint16_t>(response[6]);

  uint16_t position = (position_high_byte << 8) | position_low_byte;
  return position;
}

}  // namespace robot::perception
