#include "robot/board/teensy/teensy_board.h"

#include <memory>

#include "absl/status/status.h"
#include "firmware/common/joshua_wire_v1.h"
#include "gtest/gtest.h"
#include "robot/board/frame/fake_frame_transport.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"

// TeensyBoard only supplies two facts to the shared JoshuaWireBoard, as
// constructor arguments (BoardType::TEENSY41, JW1_BOARD_TEENSY41) —
// everything else (IDENTIFY handshake, CONFIGURE_CHANNEL, multi-channel,
// ENABLE/SET_TARGET/GET_FEEDBACK, ...) is generic protocol orchestration,
// tested once in robot/board/joshua_wire/joshua_wire_board_test.cc. These
// tests only prove Teensy's identity is wired to the right values.
namespace robot::board {
namespace {

std::vector<uint8_t> MakeIdentifyResponse(uint8_t n_channels,
                                          jw1_board_id_t board_id = JW1_BOARD_TEENSY41) {
  jw1_identify_response_t response{};
  response.board_id = board_id;
  response.n_channels = n_channels;
  for (uint8_t i = 0; i < n_channels; i++) {
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

robot::board::Board MakeTeensyBoard() {
  robot::board::Board board;
  board.set_name("stepper_bus");
  board.set_board_type(robot::board::BoardType::TEENSY41);
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
  channel->mutable_step_dir()->set_invert_dir(false);
  channel->mutable_step_dir()->set_enable_active_low(true);
  channel->mutable_step_dir()->set_step_pin(2);
  channel->mutable_step_dir()->set_dir_pin(3);
  channel->mutable_step_dir()->set_enable_pin(4);
  return board;
}

class TeensyBoardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_shared<FakeFrameTransport>();
    TeensyBoard::SetFrameTransportFactoryForTesting(
        [this](const robot::comm::Comm&) -> absl::StatusOr<std::shared_ptr<FrameTransport>> {
          return transport_;
        });
  }

  void TearDown() override {
    TeensyBoard::SetFrameTransportFactoryForTesting(nullptr);
  }

  std::shared_ptr<FakeFrameTransport> transport_;
};

TEST_F(TeensyBoardTest, InitSucceedsAgainstRealTeensyIdentity) {
  transport_->QueueResponse(MakeIdentifyResponse(1));
  transport_->QueueResponse(MakeStatusResponse(JW1_CMD_CONFIGURE_CHANNEL, 0, JW1_STATUS_OK));
  TeensyBoard board;

  EXPECT_TRUE(board.Init(MakeTeensyBoard()).ok());
}

TEST_F(TeensyBoardTest, InitRejectsNonTeensyBoardType) {
  auto config = MakeTeensyBoard();
  config.set_board_type(robot::board::BoardType::ARDUINO_UNO);
  TeensyBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(TeensyBoardTest, InitRejectsNonTeensyWireBoardId) {
  // Proves TeensyBoard's constructor actually passed JW1_BOARD_TEENSY41
  // through to JoshuaWireBoard — e.g. a re-enumerated serial path now
  // pointing at an Arduino instead (docs/BOARD_LAYER_RFC.md §7.5).
  transport_->QueueResponse(MakeIdentifyResponse(1, JW1_BOARD_ARDUINO_UNO));
  TeensyBoard board;

  EXPECT_EQ(board.Init(MakeTeensyBoard()).code(), absl::StatusCode::kFailedPrecondition);
}

}  // namespace
}  // namespace robot::board
