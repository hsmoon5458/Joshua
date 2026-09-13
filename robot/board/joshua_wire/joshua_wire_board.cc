#include "robot/board/joshua_wire/joshua_wire_board.h"

#include <set>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "robot/comm/factory/comm_factory.h"
#include "utils/status_macros.h"

namespace robot::board {

namespace {

std::function<absl::StatusOr<std::shared_ptr<FrameTransport>>(const robot::comm::Comm&)>&
FrameTransportFactoryForTesting() {
  static std::function<absl::StatusOr<std::shared_ptr<FrameTransport>>(const robot::comm::Comm&)>
      factory;
  return factory;
}

absl::Status JwStatusToAbsl(jw1_status_t status, const std::string& what) {
  switch (status) {
    case JW1_STATUS_OK:
      return absl::OkStatus();
    case JW1_STATUS_UNSUPPORTED:
      return absl::UnimplementedError(absl::StrCat(what, ": firmware reports unsupported."));
    case JW1_STATUS_ERROR:
    default:
      return absl::InternalError(absl::StrCat(what, ": firmware reports error."));
  }
}

// robot.board.DriveInterface -> jw1_drive_t, value-for-value (mirrors the
// comment on jw1_drive_t in joshua_wire_v1.h). Used to cross-check IDENTIFY's
// reported per-channel drive against what config declares, generically —
// adding a wire-side drive here (there already are PWM_DC/SERVO_BUS_UART/
// CAN/PDO_JOINT slots reserved) needs no change to IdentifyAndValidate.
absl::StatusOr<jw1_drive_t> ToWireDrive(robot::board::DriveInterface drive) {
  switch (drive) {
    case robot::board::DriveInterface::STEP_DIR:
      return JW1_DRIVE_STEP_DIR;
    case robot::board::DriveInterface::PWM_DC:
      return JW1_DRIVE_PWM_DC;
    case robot::board::DriveInterface::SERVO_BUS_UART:
      return JW1_DRIVE_SERVO_BUS_UART;
    case robot::board::DriveInterface::CAN:
      return JW1_DRIVE_CAN;
    case robot::board::DriveInterface::PDO_JOINT:
      return JW1_DRIVE_PDO_JOINT;
    default:
      return absl::InvalidArgumentError(absl::StrCat("DriveInterface ",
                                                     robot::board::DriveInterface_Name(drive),
                                                     " has no joshua_wire_v1 wire-drive mapping."));
  }
}

// One addressable channel over a joshua_wire_v1 frame transport.
class JoshuaWireChannel : public BoardChannel {
 public:
  JoshuaWireChannel(std::shared_ptr<FrameTransport> transport, uint8_t channel_index)
      : transport_(std::move(transport)), channel_index_(channel_index) {}

  absl::Status Enable() override {
    uint8_t buf[JW1_MAX_FRAME_LEN];
    const int len = jw1_encode_enable(buf, sizeof(buf), channel_index_);
    return SendExpectStatus(buf, len, "Enable");
  }

  absl::Status Disable() override {
    uint8_t buf[JW1_MAX_FRAME_LEN];
    const int len = jw1_encode_disable(buf, sizeof(buf), channel_index_);
    return SendExpectStatus(buf, len, "Disable");
  }

  absl::Status SetTarget(TargetMode mode, float value) override {
    if (mode == TargetMode::kTorque) {
      // The protocol does not define a continuous torque target.
      return absl::UnimplementedError(
          "joshua_wire_v1 channel has no torque target (open-loop drive).");
    }
    const jw1_mode_t wire_mode =
        mode == TargetMode::kPosition ? JW1_MODE_POSITION : JW1_MODE_VELOCITY;
    uint8_t buf[JW1_MAX_FRAME_LEN];
    const int len = jw1_encode_set_target(buf, sizeof(buf), channel_index_, wire_mode, value);
    return SendExpectStatus(buf, len, "SetTarget");
  }

  absl::StatusOr<ChannelFeedback> ReadFeedback() override {
    uint8_t buf[JW1_MAX_FRAME_LEN];
    const int len = jw1_encode_get_feedback_request(buf, sizeof(buf), channel_index_);
    if (len < 0) {
      return absl::InternalError("Failed to encode GET_FEEDBACK request.");
    }
    ABSL_ASSIGN_OR_RETURN(
        auto response,
        transport_->SendAndReceive(std::vector<uint8_t>(buf, buf + len),
                                   JW1_FRAME_LEN(JW1_FEEDBACK_RESPONSE_PAYLOAD_LEN)));
    jw1_frame_t frame;
    if (jw1_decode_frame(response.data(), response.size(), &frame) != 0) {
      return absl::InternalError("Malformed GET_FEEDBACK response frame.");
    }
    jw1_feedback_t feedback;
    if (jw1_decode_feedback_response(&frame, &feedback) != 0) {
      return absl::InternalError("Malformed GET_FEEDBACK response payload.");
    }
    ChannelFeedback out;
    out.position = feedback.position;
    out.velocity = feedback.velocity;
    out.fault_flags = feedback.fault_flags;
    return out;
  }

 private:
  absl::Status SendExpectStatus(const uint8_t* buf, int len, const std::string& what) {
    if (len < 0) {
      return absl::InternalError(absl::StrCat("Failed to encode ", what, " request."));
    }
    ABSL_ASSIGN_OR_RETURN(
        auto response,
        transport_->SendAndReceive(std::vector<uint8_t>(buf, buf + len),
                                   JW1_FRAME_LEN(JW1_STATUS_RESPONSE_PAYLOAD_LEN)));
    jw1_frame_t frame;
    if (jw1_decode_frame(response.data(), response.size(), &frame) != 0) {
      return absl::InternalError(absl::StrCat("Malformed ", what, " response frame."));
    }
    jw1_status_t status;
    if (jw1_decode_status_response(&frame, &status) != 0) {
      return absl::InternalError(absl::StrCat("Malformed ", what, " response payload."));
    }
    return JwStatusToAbsl(status, what);
  }

  std::shared_ptr<FrameTransport> transport_;
  const uint8_t channel_index_;
};

// STEP_DIR is the only drive_config case joshua_wire_v1's CONFIGURE_CHANNEL
// encoder implements today. Adding a second (e.g. PWM) means adding a case
// to ConfigureChannel's switch below plus a jw1_encode_configure_channel_*
// in firmware/common/ — no change anywhere else in this file, or in any
// board that subclasses JoshuaWireBoard.
absl::Status ConfigureStepDirChannel(FrameTransport& transport,
                                     const robot::board::Channel& channel) {
  jw1_configure_step_dir_t config{};
  config.max_pulse_rate_hz = channel.step_dir().max_pulse_rate_hz();
  config.invert_dir = channel.step_dir().invert_dir() ? 1 : 0;
  config.enable_active_low = channel.step_dir().enable_active_low() ? 1 : 0;
  config.step_pin = static_cast<uint8_t>(channel.step_dir().step_pin());
  config.dir_pin = static_cast<uint8_t>(channel.step_dir().dir_pin());
  config.enable_pin = static_cast<uint8_t>(channel.step_dir().enable_pin());
  config.step_pulse_width_us = static_cast<uint16_t>(channel.step_dir().step_pulse_width_us());

  uint8_t buf[JW1_MAX_FRAME_LEN];
  const int len = jw1_encode_configure_channel_step_dir(
      buf, sizeof(buf), static_cast<uint8_t>(channel.index()), &config);
  if (len < 0) {
    return absl::InternalError("Failed to encode CONFIGURE_CHANNEL request.");
  }
  ABSL_ASSIGN_OR_RETURN(auto response,
                        transport.SendAndReceive(std::vector<uint8_t>(buf, buf + len),
                                                 JW1_FRAME_LEN(JW1_STATUS_RESPONSE_PAYLOAD_LEN)));
  jw1_frame_t frame;
  if (jw1_decode_frame(response.data(), response.size(), &frame) != 0) {
    return absl::InternalError("Malformed CONFIGURE_CHANNEL response frame.");
  }
  jw1_status_t status;
  if (jw1_decode_status_response(&frame, &status) != 0) {
    return absl::InternalError("Malformed CONFIGURE_CHANNEL response payload.");
  }
  return JwStatusToAbsl(status, absl::StrCat("CONFIGURE_CHANNEL(", channel.index(), ")"));
}

absl::Status ConfigureChannel(FrameTransport& transport, const robot::board::Channel& channel) {
  switch (channel.drive_config_case()) {
    case robot::board::Channel::kStepDir:
      return ConfigureStepDirChannel(transport, channel);
    default:
      // ValidateConfig already rejects this before Init() ever opens a
      // transport; reachable only if a future caller skips that check.
      return absl::UnimplementedError(
          absl::StrCat("channel ",
                       channel.index(),
                       ": joshua_wire_v1's CONFIGURE_CHANNEL encoder has no support for "
                       "drive_config case ",
                       static_cast<int>(channel.drive_config_case()),
                       " yet."));
  }
}

}  // namespace

absl::Status JoshuaWireBoard::ValidateComm(const robot::comm::Comm& comm,
                                           const std::string& board_name) const {
  if (comm.comm_type() != robot::comm::CommType::SERIAL || !comm.has_serial_config()) {
    return absl::InvalidArgumentError(
        absl::StrCat("Board '", board_name, "' requires SERIAL comm config."));
  }
  if (comm.transport_type() != robot::comm::TransportType::MESSAGE) {
    return absl::InvalidArgumentError(
        absl::StrCat("Board '", board_name, "' requires MESSAGE transport."));
  }
  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<FrameTransport>> JoshuaWireBoard::CreateTransport(
    const robot::comm::Comm& comm) const {
  ABSL_ASSIGN_OR_RETURN(auto transport, robot::comm::CommFactory::CreateComm(comm));
  return robot::comm::GetCommTransport<robot::comm::MessageTransport>(transport);
}

absl::Status JoshuaWireBoard::ValidateConfig(const robot::board::Board& config) const {
  const std::string type_name = robot::board::BoardType_Name(expected_board_type_);
  if (config.board_type() != expected_board_type_) {
    return absl::InvalidArgumentError(
        absl::StrCat("Board '", config.name(), "' is not a ", type_name, " board."));
  }
  ABSL_RETURN_IF_ERROR(ValidateComm(config.comm(), config.name()));
  if (!config.has_firmware()) {
    return absl::InvalidArgumentError(absl::StrCat(
        type_name, " board '", config.name(), "' requires a firmware{} spec for IDENTIFY."));
  }
  if (config.channels_size() == 0) {
    return absl::InvalidArgumentError(
        absl::StrCat(type_name, " board '", config.name(), "' declares no channels."));
  }
  if (config.channels_size() > JW1_MAX_CHANNELS) {
    return absl::InvalidArgumentError(absl::StrCat(type_name,
                                                   " board '",
                                                   config.name(),
                                                   "' declares more channels than joshua_wire_v1 "
                                                   "supports (",
                                                   JW1_MAX_CHANNELS,
                                                   ")."));
  }
  std::set<uint32_t> seen_indices;
  for (const auto& channel : config.channels()) {
    // STEP_DIR is the only drive this codec's CONFIGURE_CHANNEL encoder
    // implements today (see ConfigureChannel below) — not a per-board
    // limit, so this check lives here rather than in a subclass.
    if (channel.drive() != robot::board::DriveInterface::STEP_DIR ||
        channel.drive_config_case() != robot::board::Channel::kStepDir) {
      return absl::InvalidArgumentError(
          absl::StrCat(type_name,
                       " board '",
                       config.name(),
                       "' channel ",
                       channel.index(),
                       " declares drive ",
                       robot::board::DriveInterface_Name(channel.drive()),
                       ", but joshua_wire_v1's CONFIGURE_CHANNEL encoder only supports "
                       "STEP_DIR today."));
    }
    for (uint32_t pin : {channel.step_dir().step_pin(),
                         channel.step_dir().dir_pin(),
                         channel.step_dir().enable_pin()}) {
      if (pin > 255) {
        return absl::InvalidArgumentError(
            absl::StrCat(type_name,
                         " board '",
                         config.name(),
                         "' channel ",
                         channel.index(),
                         " declares a pin number ",
                         pin,
                         " that doesn't fit an MCU GPIO pin (max 255)."));
      }
    }
    if (channel.step_dir().step_pulse_width_us() > 65535) {
      return absl::InvalidArgumentError(
          absl::StrCat(type_name,
                       " board '",
                       config.name(),
                       "' channel ",
                       channel.index(),
                       " declares step_pulse_width_us ",
                       channel.step_dir().step_pulse_width_us(),
                       " that doesn't fit the wire field (max 65535)."));
    }
    if (channel.index() >= JW1_MAX_CHANNELS) {
      return absl::InvalidArgumentError(absl::StrCat(type_name,
                                                     " board '",
                                                     config.name(),
                                                     "' channel index ",
                                                     channel.index(),
                                                     " exceeds joshua_wire_v1's max channel "
                                                     "index (",
                                                     JW1_MAX_CHANNELS - 1,
                                                     "); it must match the firmware channel "
                                                     "table's array position."));
    }
    if (!seen_indices.insert(channel.index()).second) {
      return absl::InvalidArgumentError(absl::StrCat(type_name,
                                                     " board '",
                                                     config.name(),
                                                     "' declares channel index ",
                                                     channel.index(),
                                                     " more than once."));
    }
  }
  return absl::OkStatus();
}

// Validates the board identity, protocol version, and channel capabilities
// reported by firmware against configuration.
absl::Status JoshuaWireBoard::IdentifyAndValidate(FrameTransport& transport,
                                                  const robot::board::Board& config) const {
  uint8_t request[JW1_MAX_FRAME_LEN];
  const int request_len = jw1_encode_identify_request(request, sizeof(request));
  if (request_len < 0) {
    return absl::InternalError("Failed to encode IDENTIFY request.");
  }
  ABSL_ASSIGN_OR_RETURN(
      auto response,
      transport.SendAndReceive(std::vector<uint8_t>(request, request + request_len),
                               JW1_FRAME_LEN(JW1_IDENTIFY_RESPONSE_PAYLOAD_LEN)));

  jw1_frame_t frame;
  if (jw1_decode_frame(response.data(), response.size(), &frame) != 0) {
    return absl::UnavailableError(absl::StrCat(
        "Board '", config.name(), "': malformed IDENTIFY response; check wiring/firmware."));
  }
  if (frame.proto_ver < config.firmware().min_proto_version()) {
    return absl::FailedPreconditionError(absl::StrCat("Board '",
                                                      config.name(),
                                                      "': firmware proto_ver ",
                                                      frame.proto_ver,
                                                      " is older than the configured "
                                                      "min_proto_version ",
                                                      config.firmware().min_proto_version(),
                                                      "."));
  }

  jw1_identify_response_t identify;
  if (jw1_decode_identify_response(&frame, &identify) != 0) {
    return absl::UnavailableError(
        absl::StrCat("Board '", config.name(), "': malformed IDENTIFY payload."));
  }
  // board_id mirrors BoardType value-for-value (joshua_wire_v1.h) precisely
  // so a host can catch "wrong device on this port" — e.g. a stale/
  // re-enumerated serial path now pointing at a different board type —
  // instead of silently proceeding as long as channel shapes happen to
  // match.
  if (identify.board_id != expected_wire_board_id_) {
    return absl::FailedPreconditionError(absl::StrCat("Board '",
                                                      config.name(),
                                                      "': IDENTIFY reports board_id ",
                                                      static_cast<int>(identify.board_id),
                                                      ", expected ",
                                                      static_cast<int>(expected_wire_board_id_),
                                                      ". Wrong device on this port?"));
  }

  for (const auto& channel : config.channels()) {
    if (channel.index() >= identify.n_channels) {
      return absl::FailedPreconditionError(absl::StrCat("Board '",
                                                        config.name(),
                                                        "': config declares channel ",
                                                        channel.index(),
                                                        " but firmware IDENTIFY reports only ",
                                                        identify.n_channels,
                                                        " channels."));
    }
    ABSL_ASSIGN_OR_RETURN(const jw1_drive_t expected_drive, ToWireDrive(channel.drive()));
    if (identify.channel_drives[channel.index()] != expected_drive) {
      return absl::FailedPreconditionError(
          absl::StrCat("Board '",
                       config.name(),
                       "': config declares channel ",
                       channel.index(),
                       " as ",
                       robot::board::DriveInterface_Name(channel.drive()),
                       ", but firmware IDENTIFY reports a "
                       "different drive."));
    }
  }
  return absl::OkStatus();
}

absl::Status JoshuaWireBoard::Init(const robot::board::Board& config) {
  if (initialized_) {
    return absl::FailedPreconditionError(
        absl::StrCat("Board '", config.name(), "' is already initialized."));
  }
  ABSL_RETURN_IF_ERROR(ValidateConfig(config));

  std::shared_ptr<FrameTransport> transport;
  if (FrameTransportFactoryForTesting()) {
    ABSL_ASSIGN_OR_RETURN(transport, FrameTransportFactoryForTesting()(config.comm()));
  } else {
    ABSL_ASSIGN_OR_RETURN(transport, CreateTransport(config.comm()));
  }

  ABSL_RETURN_IF_ERROR(IdentifyAndValidate(*transport, config));

  std::map<uint32_t, std::shared_ptr<BoardChannel>> channels;
  for (const auto& channel_config : config.channels()) {
    ABSL_RETURN_IF_ERROR(ConfigureChannel(*transport, channel_config));
    channels[channel_config.index()] = std::make_shared<JoshuaWireChannel>(
        transport, static_cast<uint8_t>(channel_config.index()));
  }

  config_ = config;
  transport_ = std::move(transport);
  channels_ = std::move(channels);
  initialized_ = true;
  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<BoardChannel>> JoshuaWireBoard::OpenChannel(uint32_t index) {
  if (!initialized_) {
    return absl::FailedPreconditionError("Board is not initialized.");
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

absl::Status JoshuaWireBoard::Teardown() {
  channels_.clear();
  transport_.reset();
  initialized_ = false;
  return absl::OkStatus();
}

void JoshuaWireBoard::SetFrameTransportFactoryForTesting(
    std::function<absl::StatusOr<std::shared_ptr<FrameTransport>>(const robot::comm::Comm&)>
        factory) {
  FrameTransportFactoryForTesting() = std::move(factory);
}

}  // namespace robot::board
