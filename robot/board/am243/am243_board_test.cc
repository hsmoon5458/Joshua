#include "robot/board/am243/am243_board.h"

#include <memory>
#include <vector>

#include "absl/status/status.h"
#include "firmware/common/joshua_wire_v1.h"
#include "gtest/gtest.h"
#include "robot/board/frame/fake_frame_transport.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/ethercat/fake_ethercat_transport.h"
#include "robot/comm/factory/comm_factory.h"
#include "robot/comm/proto/comm.pb.h"

namespace robot::board {
namespace {

std::vector<uint8_t> MakeIdentifyResponse(uint8_t n_channels,
                                          jw1_board_id_t board_id = JW1_BOARD_AM243) {
  jw1_identify_response_t response{};
  response.board_id = board_id;
  response.n_channels = n_channels;
  for (uint8_t i = 0; i < n_channels; ++i) {
    response.channel_drives[i] = JW1_DRIVE_STEP_DIR;
  }
  uint8_t buf[JW1_MAX_FRAME_LEN];
  const int len = jw1_encode_identify_response(buf, sizeof(buf), &response);
  return std::vector<uint8_t>(buf, buf + len);
}

std::vector<uint8_t> MakeStatusResponse(uint8_t cmd, uint8_t channel, jw1_status_t status) {
  uint8_t buf[JW1_MAX_FRAME_LEN];
  const int len = jw1_encode_status_response(buf, sizeof(buf), cmd, channel, status);
  return std::vector<uint8_t>(buf, buf + len);
}

robot::board::Board MakeAm243Board() {
  robot::board::Board board;
  board.set_name("am243_stepper_bus");
  board.set_board_type(robot::board::BoardType::AM243);
  auto* comm = board.mutable_comm();
  comm->set_comm_type(robot::comm::CommType::SERIAL);
  comm->set_transport_type(robot::comm::TransportType::MESSAGE);
  comm->mutable_serial_config()->set_port("/dev/ttyACM0");
  comm->mutable_serial_config()->set_baudrate(115200);
  board.mutable_firmware()->set_min_proto_version(1);

  auto* channel = board.add_channels();
  channel->set_index(0);
  channel->set_drive(robot::board::DriveInterface::STEP_DIR);
  channel->mutable_step_dir()->set_max_pulse_rate_hz(20000);
  channel->mutable_step_dir()->set_enable_active_low(true);
  channel->mutable_step_dir()->set_step_pin(2);
  channel->mutable_step_dir()->set_dir_pin(3);
  channel->mutable_step_dir()->set_enable_pin(4);
  return board;
}

robot::board::Board MakeAm243EthercatBoard() {
  robot::board::Board board;
  board.set_name("am243_ethercat");
  board.set_board_type(robot::board::BoardType::AM243);
  auto* comm = board.mutable_comm();
  comm->set_comm_type(robot::comm::CommType::ETHERCAT);
  comm->set_transport_type(robot::comm::TransportType::CYCLIC);
  comm->mutable_ethercat_config()->set_interface_name("fake-am243-iface0");
  comm->mutable_ethercat_config()->set_process_data_mode(
      robot::comm::EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_SPLIT_LRD_LWR);
  auto* channel = board.add_channels();
  channel->set_index(0);
  channel->set_drive(robot::board::DriveInterface::PDO_JOINT);
  auto* am243 = board.mutable_am243_config();
  am243->set_slave_index(2);
  am243->set_pdo_mapping(robot::board::Am243PdoMapping::AM243_PDO_MAPPING_TI_DEMO);
  am243->set_output_offset_bytes(4);
  am243->set_input_offset_bytes(12);
  am243->set_output_size_bytes(8);
  am243->set_input_size_bytes(8);
  return board;
}

class Am243BoardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    serial_transport_ = std::make_shared<FakeFrameTransport>();
    Am243Board::SetFrameTransportFactoryForTesting(
        [this](const robot::comm::Comm&) -> absl::StatusOr<std::shared_ptr<FrameTransport>> {
          return serial_transport_;
        });
    ethercat_transport_ = std::make_shared<robot::comm::ethercat::FakeEthercatTransport>();
    robot::comm::CommFactory::SetEthercatTransportFactoryForTesting(
        [this] { return ethercat_transport_; });
  }

  void TearDown() override {
    Am243Board::SetFrameTransportFactoryForTesting(nullptr);
    robot::comm::CommFactory::SetEthercatTransportFactoryForTesting(nullptr);
    robot::comm::CommFactory::ResetEthercatTransportCacheForTesting();
  }

  std::shared_ptr<FakeFrameTransport> serial_transport_;
  std::shared_ptr<robot::comm::ethercat::FakeEthercatTransport> ethercat_transport_;
};

TEST_F(Am243BoardTest, InitSucceedsAgainstAm243Identity) {
  serial_transport_->QueueResponse(MakeIdentifyResponse(1));
  serial_transport_->QueueResponse(MakeStatusResponse(JW1_CMD_CONFIGURE_CHANNEL, 0, JW1_STATUS_OK));
  Am243Board board;

  EXPECT_TRUE(board.Init(MakeAm243Board()).ok());
}

TEST_F(Am243BoardTest, InitRejectsNonAm243BoardType) {
  auto config = MakeAm243Board();
  config.set_board_type(robot::board::BoardType::TEENSY41);
  Am243Board board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(Am243BoardTest, InitSupportsEthercatDemoConfig) {
  Am243Board board;

  EXPECT_TRUE(board.Init(MakeAm243EthercatBoard()).ok());
  EXPECT_EQ(ethercat_transport_->configure_slaves_calls_, 1);
  EXPECT_EQ(ethercat_transport_->start_cyclic_calls_, 1);
}

TEST_F(Am243BoardTest, InitRejectsNonAm243WireIdentity) {
  serial_transport_->QueueResponse(MakeIdentifyResponse(1, JW1_BOARD_TEENSY41));
  Am243Board board;

  EXPECT_EQ(board.Init(MakeAm243Board()).code(), absl::StatusCode::kFailedPrecondition);
}

}  // namespace
}  // namespace robot::board
