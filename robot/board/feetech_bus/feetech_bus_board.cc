#include "robot/board/feetech_bus/feetech_bus_board.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <set>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "robot/board/feetech_bus/feetech_protocol.h"
#include "robot/comm/factory/comm_factory.h"
#include "robot/comm/proto/comm.pb.h"
#include "utils/status_macros.h"

namespace robot::board {

struct FeetechBusSharedState {
  std::shared_ptr<robot::comm::MessageTransport> transport;
  // Allows one request/response exchange at a time on the half-duplex bus.
  std::mutex bus_mutex;
};

namespace {

std::function<
    absl::StatusOr<std::shared_ptr<robot::comm::MessageTransport>>(const robot::comm::Comm&)>&
SerialTransportFactoryForTesting() {
  static std::function<absl::StatusOr<std::shared_ptr<robot::comm::MessageTransport>>(
      const robot::comm::Comm&)>
      factory;
  return factory;
}

// One addressable channel on a shared servo bus.
class FeetechBusChannel : public BoardChannel {
 public:
  FeetechBusChannel(std::shared_ptr<FeetechBusSharedState> state,
                    uint8_t servo_id,
                    uint16_t move_time_ms)
      : state_(std::move(state)), servo_id_(servo_id), move_time_ms_(move_time_ms) {}

  absl::Status Enable() override {
    return WriteTorqueEnable(true);
  }

  absl::Status Disable() override {
    return WriteTorqueEnable(false);
  }

  absl::Status SetTarget(TargetMode mode, float value) override {
    switch (mode) {
      case TargetMode::kVelocity: {
        std::lock_guard<std::mutex> lock(mutex_);
        staged_move_speed_ = ClampToUint16(value);
        return absl::OkStatus();
      }
      case TargetMode::kPosition: {
        uint16_t staged_speed;
        {
          std::lock_guard<std::mutex> lock(mutex_);
          staged_speed = staged_move_speed_;
        }
        std::vector<uint8_t> data = feetech::EncodeUint16Le(ClampToUint16(value));
        auto time_bytes = feetech::EncodeUint16Le(move_time_ms_);
        auto speed_bytes = feetech::EncodeUint16Le(staged_speed);
        data.insert(data.end(), time_bytes.begin(), time_bytes.end());
        data.insert(data.end(), speed_bytes.begin(), speed_bytes.end());
        const auto packet = feetech::BuildWritePacket(servo_id_, feetech::kRegGoalPosition, data);
        std::lock_guard<std::mutex> bus_lock(state_->bus_mutex);
        return state_->transport->Write(packet);
      }
      case TargetMode::kTorque:
        // This bus exposes torque enable, but no continuous torque target.
        return absl::UnimplementedError(
            "FEETECH_BUS channel has no continuous torque target; use Enable/Disable.");
    }
    return absl::UnimplementedError("Unknown target mode.");
  }

  absl::StatusOr<ChannelFeedback> ReadFeedback() override {
    const auto request = feetech::BuildReadPacket(servo_id_, feetech::kRegPresentPosition, 2);
    std::vector<uint8_t> response;
    {
      std::lock_guard<std::mutex> lock(state_->bus_mutex);
      ABSL_ASSIGN_OR_RETURN(
          response,
          state_->transport->SendAndReceive(request, feetech::kStatusPacketOverheadBytes + 2));
    }
    ABSL_ASSIGN_OR_RETURN(auto params, feetech::ParseStatusPacket(response, servo_id_));
    if (params.size() != 2) {
      return absl::InternalError(absl::StrCat("Servo ",
                                              static_cast<int>(servo_id_),
                                              " present-position read returned ",
                                              params.size(),
                                              " bytes, expected 2."));
    }
    ChannelFeedback feedback;
    feedback.position = static_cast<float>(feetech::DecodeUint16Le(params[0], params[1]));
    return feedback;
  }

 private:
  static uint16_t ClampToUint16(float value) {
    return static_cast<uint16_t>(std::clamp(std::lround(value), 0L, 65535L));
  }

  absl::Status WriteTorqueEnable(bool enable) {
    const auto packet = feetech::BuildWritePacket(
        servo_id_, feetech::kRegTorqueEnable, {static_cast<uint8_t>(enable ? 1 : 0)});
    std::lock_guard<std::mutex> lock(state_->bus_mutex);
    return state_->transport->Write(packet);
  }

  std::shared_ptr<FeetechBusSharedState> state_;
  const uint8_t servo_id_;
  const uint16_t move_time_ms_;
  std::mutex mutex_;
  uint16_t staged_move_speed_ = 0;
};

absl::Status ValidateConfig(const robot::board::Board& config) {
  if (config.board_type() != robot::board::BoardType::FEETECH_BUS) {
    return absl::InvalidArgumentError(
        absl::StrCat("Board '", config.name(), "' is not a FEETECH_BUS board."));
  }
  if (config.comm().comm_type() != robot::comm::CommType::SERIAL ||
      !config.comm().has_serial_config()) {
    return absl::InvalidArgumentError(
        absl::StrCat("FEETECH_BUS board '", config.name(), "' requires SERIAL comm config."));
  }
  if (config.comm().transport_type() != robot::comm::TransportType::MESSAGE) {
    return absl::InvalidArgumentError(
        absl::StrCat("FEETECH_BUS board '", config.name(), "' requires MESSAGE transport."));
  }
  if (config.channels_size() == 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("FEETECH_BUS board '", config.name(), "' declares no channels."));
  }
  std::set<uint32_t> seen_servo_ids;
  for (const auto& channel : config.channels()) {
    if (channel.drive() != robot::board::DriveInterface::SERVO_BUS_UART) {
      return absl::InvalidArgumentError(absl::StrCat("FEETECH_BUS board '",
                                                     config.name(),
                                                     "' channel ",
                                                     channel.index(),
                                                     " must use the SERVO_BUS_UART drive."));
    }
    if (!channel.has_servo_bus() || channel.servo_bus().servo_id() == 0) {
      return absl::InvalidArgumentError(absl::StrCat("FEETECH_BUS board '",
                                                     config.name(),
                                                     "' channel ",
                                                     channel.index(),
                                                     " has no servo_bus.servo_id."));
    }
    if (!seen_servo_ids.insert(channel.servo_bus().servo_id()).second) {
      return absl::InvalidArgumentError(absl::StrCat("FEETECH_BUS board '",
                                                     config.name(),
                                                     "' declares servo_id ",
                                                     channel.servo_bus().servo_id(),
                                                     " more than once."));
    }
  }
  return absl::OkStatus();
}

// Verifies that the configured servo responds and exposes its model register.
absl::Status IdentifyServo(FeetechBusSharedState& state,
                           const std::string& board_name,
                           uint8_t servo_id) {
  std::lock_guard<std::mutex> lock(state.bus_mutex);

  const auto ping = feetech::BuildPingPacket(servo_id);
  ABSL_ASSIGN_OR_RETURN(auto ping_response,
                        state.transport->SendAndReceive(ping, feetech::kStatusPacketOverheadBytes));
  auto ping_status = feetech::ParseStatusPacket(ping_response, servo_id);
  if (!ping_status.ok()) {
    return absl::UnavailableError(absl::StrCat("Board '",
                                               board_name,
                                               "': no response from servo ",
                                               static_cast<int>(servo_id),
                                               " on the Feetech bus; check wiring/servo ID. ",
                                               ping_status.status().message()));
  }

  const auto read_model = feetech::BuildReadPacket(servo_id, feetech::kRegModelNumber, 2);
  ABSL_ASSIGN_OR_RETURN(
      auto model_response,
      state.transport->SendAndReceive(read_model, feetech::kStatusPacketOverheadBytes + 2));
  ABSL_RETURN_IF_ERROR(feetech::ParseStatusPacket(model_response, servo_id).status());
  return absl::OkStatus();
}

}  // namespace

absl::Status FeetechBusBoard::Init(const robot::board::Board& config) {
  if (initialized_) {
    return absl::FailedPreconditionError(
        absl::StrCat("FEETECH_BUS board '", config.name(), "' is already initialized."));
  }
  ABSL_RETURN_IF_ERROR(ValidateConfig(config));

  auto state = std::make_shared<FeetechBusSharedState>();
  if (SerialTransportFactoryForTesting()) {
    ABSL_ASSIGN_OR_RETURN(state->transport, SerialTransportFactoryForTesting()(config.comm()));
  } else {
    ABSL_ASSIGN_OR_RETURN(auto comm, robot::comm::CommFactory::CreateComm(config.comm()));
    ABSL_ASSIGN_OR_RETURN(state->transport,
                          robot::comm::GetCommTransport<robot::comm::MessageTransport>(comm));
  }

  std::map<uint32_t, std::shared_ptr<BoardChannel>> channels;
  for (const auto& channel_config : config.channels()) {
    if (channels.count(channel_config.index()) > 0) {
      return absl::InvalidArgumentError(absl::StrCat("Board '",
                                                     config.name(),
                                                     "' declares channel index ",
                                                     channel_config.index(),
                                                     " more than once."));
    }
    const auto servo_id = static_cast<uint8_t>(channel_config.servo_bus().servo_id());
    ABSL_RETURN_IF_ERROR(IdentifyServo(*state, config.name(), servo_id));
    channels[channel_config.index()] = std::make_shared<FeetechBusChannel>(
        state, servo_id, static_cast<uint16_t>(channel_config.servo_bus().move_time_ms()));
  }

  config_ = config;
  state_ = std::move(state);
  channels_ = std::move(channels);
  initialized_ = true;
  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<BoardChannel>> FeetechBusBoard::OpenChannel(uint32_t index) {
  if (!initialized_) {
    return absl::FailedPreconditionError("FEETECH_BUS board is not initialized.");
  }
  auto it = channels_.find(index);
  if (it == channels_.end()) {
    return absl::NotFoundError(absl::StrCat("Board '",
                                            config_.name(),
                                            "' has no channel ",
                                            index,
                                            "; declare it in the board's channels{}."));
  }
  return it->second;
}

absl::Status FeetechBusBoard::Teardown() {
  channels_.clear();
  state_.reset();
  initialized_ = false;
  return absl::OkStatus();
}

void FeetechBusBoard::SetSerialTransportFactoryForTesting(
    std::function<absl::StatusOr<std::shared_ptr<robot::comm::MessageTransport>>(
        const robot::comm::Comm&)> factory) {
  SerialTransportFactoryForTesting() = std::move(factory);
}

}  // namespace robot::board
