#include "robot/board/am243/am243_board.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <vector>

#include "absl/strings/str_cat.h"
#include "firmware/common/joshua_wire_v1.h"
#include "robot/board/am243/am243_pdo_codec.h"
#include "robot/comm/ethercat/ethercat_status.h"
#include "robot/comm/ethercat/ethercat_transport.h"
#include "robot/comm/factory/comm_factory.h"
#include "utils/status_macros.h"

namespace robot::board {

struct Am243SharedState {
  std::shared_ptr<robot::comm::ethercat::EthercatTransport> transport;
  robot::comm::ethercat::PdoRegion pdo_region;
  std::mutex bus_mutex;
};

namespace {

class Am243DemoChannel : public BoardChannel {
 public:
  explicit Am243DemoChannel(std::shared_ptr<Am243SharedState> state) : state_(std::move(state)) {}

  absl::Status Enable() override {
    return absl::OkStatus();
  }

  absl::Status Disable() override {
    return absl::OkStatus();
  }

  absl::Status SetTarget(TargetMode mode, float value) override {
    (void)mode;
    const uint8_t seed = static_cast<uint8_t>(std::clamp(std::lround(value), 0L, 255L));
    const std::vector<uint8_t> outputs = am243::EncodeDemoOutputSeed(seed);

    std::lock_guard<std::mutex> lock(state_->bus_mutex);
    ABSL_RETURN_IF_ERROR(state_->transport->WriteOutputs(state_->pdo_region, outputs));
    ABSL_ASSIGN_OR_RETURN(auto process_data, state_->transport->ExchangeProcessData());
    return robot::comm::ethercat::ValidateProcessData(process_data);
  }

  absl::StatusOr<ChannelFeedback> ReadFeedback() override {
    std::lock_guard<std::mutex> lock(state_->bus_mutex);
    ABSL_ASSIGN_OR_RETURN(auto inputs, state_->transport->ReadInputs(state_->pdo_region));
    ABSL_ASSIGN_OR_RETURN(uint8_t seed, am243::DecodeDemoInputEchoSeed(inputs));
    ChannelFeedback feedback;
    feedback.position = static_cast<float>(seed);
    return feedback;
  }

 private:
  std::shared_ptr<Am243SharedState> state_;
};

absl::Status ValidateEthercatConfig(const robot::board::Board& config) {
  if (config.board_type() != robot::board::BoardType::AM243) {
    return absl::InvalidArgumentError(
        absl::StrCat("Board '", config.name(), "' is not an AM243 board."));
  }
  if (!config.comm().has_ethercat_config()) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' has no EtherCAT comm config."));
  }
  if (config.comm().transport_type() != robot::comm::TransportType::CYCLIC) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' requires CYCLIC transport."));
  }
  if (config.comm().ethercat_config().process_data_mode() !=
      robot::comm::EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_SPLIT_LRD_LWR) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' requires split LRD/LWR process data."));
  }
  if (!config.has_am243_config()) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' has no am243_config."));
  }
  if (config.am243_config().pdo_mapping() !=
      robot::board::Am243PdoMapping::AM243_PDO_MAPPING_TI_DEMO) {
    return absl::UnimplementedError(absl::StrCat(
        "AM243 board '", config.name(), "' currently supports only the TI demo PDO mapping."));
  }
  if (config.am243_config().slave_index() == 0) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' has no slave_index."));
  }
  for (const auto& channel : config.channels()) {
    if (channel.drive() != robot::board::DriveInterface::PDO_JOINT) {
      return absl::InvalidArgumentError(absl::StrCat("AM243 board '",
                                                     config.name(),
                                                     "' channel ",
                                                     channel.index(),
                                                     " must use the PDO_JOINT drive."));
    }
  }
  return absl::OkStatus();
}

}  // namespace

absl::Status Am243Board::Init(const robot::board::Board& config) {
  if (initialized_) {
    return absl::FailedPreconditionError(
        absl::StrCat("AM243 board '", config.name(), "' is already initialized."));
  }

  if (config.comm().comm_type() == robot::comm::CommType::SERIAL) {
    auto serial_board =
        std::make_shared<JoshuaWireBoard>(robot::board::BoardType::AM243, JW1_BOARD_AM243);
    ABSL_RETURN_IF_ERROR(serial_board->Init(config));
    config_ = config;
    serial_board_ = std::move(serial_board);
    serial_mode_ = true;
    initialized_ = true;
    return absl::OkStatus();
  }

  if (config.comm().comm_type() != robot::comm::CommType::ETHERCAT) {
    return absl::InvalidArgumentError(
        absl::StrCat("AM243 board '", config.name(), "' requires SERIAL or ETHERCAT comm config."));
  }
  ABSL_RETURN_IF_ERROR(ValidateEthercatConfig(config));

  auto state = std::make_shared<Am243SharedState>();
  ABSL_ASSIGN_OR_RETURN(auto comm, robot::comm::CommFactory::CreateComm(config.comm()));
  ABSL_ASSIGN_OR_RETURN(
      state->transport,
      robot::comm::GetCommTransport<robot::comm::ethercat::EthercatTransport>(comm));
  ABSL_RETURN_IF_ERROR(state->transport->ConfigureSlaves());
  ABSL_RETURN_IF_ERROR(state->transport->StartCyclic());

  const auto& am243_config = config.am243_config();
  auto& region = state->pdo_region;
  region.slave_index = static_cast<uint16_t>(am243_config.slave_index());
  region.output_offset_bytes = am243_config.output_offset_bytes();
  region.input_offset_bytes = am243_config.input_offset_bytes();
  region.output_size_bytes = am243_config.output_size_bytes();
  region.input_size_bytes = am243_config.input_size_bytes();
  if (region.output_size_bytes == 0 && region.input_size_bytes == 0) {
    ABSL_ASSIGN_OR_RETURN(region, state->transport->GetPdoRegion(region.slave_index));
  }
  ABSL_RETURN_IF_ERROR(robot::comm::ethercat::ValidatePdoRegion(region));

  for (const auto& channel : config.channels()) {
    if (channels_.count(channel.index()) > 0) {
      channels_.clear();
      return absl::InvalidArgumentError(absl::StrCat("Board '",
                                                     config.name(),
                                                     "' declares channel index ",
                                                     channel.index(),
                                                     " more than once."));
    }
    channels_[channel.index()] = std::make_shared<Am243DemoChannel>(state);
  }

  config_ = config;
  state_ = std::move(state);
  serial_mode_ = false;
  initialized_ = true;
  return absl::OkStatus();
}

absl::StatusOr<std::shared_ptr<BoardChannel>> Am243Board::OpenChannel(uint32_t index) {
  if (!initialized_) {
    return absl::FailedPreconditionError("AM243 board is not initialized.");
  }
  if (serial_mode_) {
    return serial_board_->OpenChannel(index);
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

absl::Status Am243Board::Teardown() {
  if (!initialized_) {
    return absl::OkStatus();
  }

  absl::Status status = absl::OkStatus();
  if (serial_mode_) {
    status = serial_board_->Teardown();
    serial_board_.reset();
  } else {
    status = state_->transport->StopCyclic();
    channels_.clear();
    state_.reset();
  }
  initialized_ = false;
  serial_mode_ = false;
  return status;
}

}  // namespace robot::board
