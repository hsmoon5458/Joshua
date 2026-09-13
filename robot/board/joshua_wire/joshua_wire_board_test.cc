#include "robot/board/joshua_wire/joshua_wire_board.h"

#include <memory>

#include "absl/status/status.h"
#include "firmware/common/joshua_wire_v1.h"
#include "gtest/gtest.h"
#include "robot/board/frame/fake_frame_transport.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"

namespace robot::board {
namespace {

// Exercises JoshuaWireBoard's generic protocol orchestration through a
// minimal concrete subclass — deliberately identifying as ARDUINO_UNO, not
// TEENSY41, so these tests can't be mistaken for "the Teensy tests in
// disguise": they prove the base class works for a board other than the
// one it was first extracted from (docs/BOARD_LAYER_RFC.md §7.3). Teensy-
// specific behavior (there is none beyond identity today) belongs in
// robot/board/teensy/teensy_board_test.cc instead.
class FakeJoshuaWireBoard : public JoshuaWireBoard {
 public:
  FakeJoshuaWireBoard()
      : JoshuaWireBoard(robot::board::BoardType::ARDUINO_UNO, JW1_BOARD_ARDUINO_UNO) {}
};

std::vector<uint8_t> MakeIdentifyResponse(uint8_t n_channels,
                                          jw1_board_id_t board_id = JW1_BOARD_ARDUINO_UNO) {
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

std::vector<uint8_t> MakeFeedbackResponse(uint8_t channel,
                                          float position,
                                          float velocity,
                                          uint16_t fault_flags) {
  jw1_feedback_t feedback{position, velocity, fault_flags};
  uint8_t buf[JW1_MAX_FRAME_LEN];
  const int len = jw1_encode_feedback_response(buf, sizeof(buf), channel, &feedback);
  return std::vector<uint8_t>(buf, buf + len);
}

void AddStepDirChannel(robot::board::Board* board,
                       uint32_t index,
                       uint32_t step_pin,
                       uint32_t dir_pin,
                       uint32_t enable_pin) {
  auto* channel = board->add_channels();
  channel->set_index(index);
  channel->set_drive(robot::board::DriveInterface::STEP_DIR);
  channel->mutable_step_dir()->set_max_pulse_rate_hz(20000);
  channel->mutable_step_dir()->set_invert_dir(false);
  channel->mutable_step_dir()->set_enable_active_low(true);
  channel->mutable_step_dir()->set_step_pin(step_pin);
  channel->mutable_step_dir()->set_dir_pin(dir_pin);
  channel->mutable_step_dir()->set_enable_pin(enable_pin);
}

robot::board::Board MakeBoardConfig() {
  robot::board::Board board;
  board.set_name("bridge_1");
  board.set_board_type(robot::board::BoardType::ARDUINO_UNO);
  auto* comm = board.mutable_comm();
  comm->set_comm_type(robot::comm::CommType::SERIAL);
  comm->set_transport_type(robot::comm::TransportType::MESSAGE);
  comm->mutable_serial_config()->set_port("/dev/ttyACM0");
  comm->mutable_serial_config()->set_baudrate(115200);
  board.mutable_firmware()->set_min_proto_version(1);

  AddStepDirChannel(&board, /*index=*/0, /*step_pin=*/2, /*dir_pin=*/3, /*enable_pin=*/4);
  return board;
}

class JoshuaWireBoardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_shared<FakeFrameTransport>();
    JoshuaWireBoard::SetFrameTransportFactoryForTesting(
        [this](const robot::comm::Comm&) -> absl::StatusOr<std::shared_ptr<FrameTransport>> {
          return transport_;
        });
  }

  void TearDown() override {
    JoshuaWireBoard::SetFrameTransportFactoryForTesting(nullptr);
  }

  // Init() always does IDENTIFY then one CONFIGURE_CHANNEL per configured
  // channel (in config.channels() order, i.e. index 0, 1, ...); tests that
  // expect Init to succeed must queue IDENTIFY plus one CONFIGURE_CHANNEL
  // reply per channel actually declared in the config passed to Init().
  void QueueSuccessfulInit(uint8_t n_channels = 1) {
    transport_->QueueResponse(MakeIdentifyResponse(n_channels));
    for (uint8_t i = 0; i < n_channels; i++) {
      transport_->QueueResponse(MakeStatusResponse(JW1_CMD_CONFIGURE_CHANNEL, i, JW1_STATUS_OK));
    }
  }

  std::shared_ptr<FakeFrameTransport> transport_;
};

TEST_F(JoshuaWireBoardTest, InitIdentifiesAndConfiguresEveryChannel) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;

  auto status = board.Init(MakeBoardConfig());

  EXPECT_TRUE(status.ok()) << status;
  EXPECT_EQ(transport_->send_calls_, 2);
}

TEST_F(JoshuaWireBoardTest, InitConfiguresMultipleChannelsOnOneBoard) {
  // Multiple motors on one board: config declares two channels on distinct
  // pins, Init() must push CONFIGURE_CHANNEL for each in order, and both
  // channels must be independently addressable afterward.
  auto config = MakeBoardConfig();  // Already has channel 0 on pins 2/3/4.
  AddStepDirChannel(&config, /*index=*/1, /*step_pin=*/5, /*dir_pin=*/6, /*enable_pin=*/7);
  QueueSuccessfulInit(/*n_channels=*/2);
  FakeJoshuaWireBoard board;

  auto status = board.Init(config);

  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(transport_->send_calls_, 3);  // 1 IDENTIFY + 2 CONFIGURE_CHANNEL.

  auto channel0 = board.OpenChannel(0);
  auto channel1 = board.OpenChannel(1);
  ASSERT_TRUE(channel0.ok());
  ASSERT_TRUE(channel1.ok());

  transport_->QueueResponse(MakeStatusResponse(JW1_CMD_ENABLE, 1, JW1_STATUS_OK));
  EXPECT_TRUE((*channel1)->Enable().ok());
  EXPECT_EQ(transport_->last_sent_[4], 1);  // Reached the wire as channel 1, not 0.
}

TEST_F(JoshuaWireBoardTest, InitRejectsWrongBoardType) {
  auto config = MakeBoardConfig();
  config.set_board_type(robot::board::BoardType::MOCK);
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitRejectsNonSerialComm) {
  auto config = MakeBoardConfig();
  config.mutable_comm()->set_comm_type(robot::comm::CommType::ETHERNET_UDP);
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitRejectsMissingFirmwareSpec) {
  auto config = MakeBoardConfig();
  config.clear_firmware();
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitRejectsNonStepDirDrive) {
  auto config = MakeBoardConfig();
  config.mutable_channels(0)->set_drive(robot::board::DriveInterface::SERVO_BUS_UART);
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitRejectsPinOutOfMcuRange) {
  auto config = MakeBoardConfig();
  config.mutable_channels(0)->mutable_step_dir()->set_step_pin(300);  // Doesn't fit uint8_t.
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitRejectsStepPulseWidthOutOfWireRange) {
  auto config = MakeBoardConfig();
  config.mutable_channels(0)->mutable_step_dir()->set_step_pulse_width_us(
      70000);  // Doesn't fit uint16_t.
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, ConfigureChannelSendsStepPulseWidthOnTheWire) {
  auto config = MakeBoardConfig();
  config.mutable_channels(0)->mutable_step_dir()->set_step_pulse_width_us(500);
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;

  ASSERT_TRUE(board.Init(config).ok());

  // CONFIGURE_CHANNEL payload: max_pulse_rate_hz(4) + invert_dir(1) +
  // enable_active_low(1) + step_pin(1) + dir_pin(1) + enable_pin(1) +
  // step_pulse_width_us(2, LE) — starts at last_sent_[5].
  ASSERT_EQ(transport_->sent_.size(), 2u);
  const auto& configure_frame = transport_->sent_[1];
  ASSERT_EQ(configure_frame[3], JW1_CMD_CONFIGURE_CHANNEL);
  EXPECT_EQ(configure_frame[14], 0xf4);  // 500 & 0xFF
  EXPECT_EQ(configure_frame[15], 0x01);  // 500 >> 8
}

TEST_F(JoshuaWireBoardTest, InitRejectsDuplicateChannelIndex) {
  auto config = MakeBoardConfig();
  auto* channel = config.add_channels();
  channel->set_index(0);  // Same index as the existing channel.
  channel->set_drive(robot::board::DriveInterface::STEP_DIR);
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(JoshuaWireBoardTest, InitFailsWhenBoardIdMismatches) {
  // board_id mirrors BoardType value-for-value specifically so Init() can
  // catch "wrong device on this port" (e.g. a re-enumerated serial path
  // now pointing at a different board) instead of silently proceeding as
  // long as channel shapes happen to match (docs/BOARD_LAYER_RFC.md §7.5).
  transport_->QueueResponse(MakeIdentifyResponse(1, JW1_BOARD_TEENSY41));
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(MakeBoardConfig()).code(), absl::StatusCode::kFailedPrecondition);
}

TEST_F(JoshuaWireBoardTest, InitFailsWhenFirmwareReportsFewerChannels) {
  transport_->QueueResponse(MakeIdentifyResponse(/*n_channels=*/0));
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(MakeBoardConfig()).code(), absl::StatusCode::kFailedPrecondition);
}

TEST_F(JoshuaWireBoardTest, InitFailsWhenConfigureChannelReturnsError) {
  transport_->QueueResponse(MakeIdentifyResponse(1));
  transport_->QueueResponse(MakeStatusResponse(JW1_CMD_CONFIGURE_CHANNEL, 0, JW1_STATUS_ERROR));
  FakeJoshuaWireBoard board;

  EXPECT_EQ(board.Init(MakeBoardConfig()).code(), absl::StatusCode::kInternal);
}

TEST_F(JoshuaWireBoardTest, OpenChannelEnableSendsEnableFrame) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());
  auto channel = board.OpenChannel(0);
  ASSERT_TRUE(channel.ok()) << channel.status();

  transport_->QueueResponse(MakeStatusResponse(JW1_CMD_ENABLE, 0, JW1_STATUS_OK));
  auto status = (*channel)->Enable();

  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(transport_->last_sent_[3], JW1_CMD_ENABLE);
  EXPECT_EQ(transport_->last_sent_[4], 0);
}

TEST_F(JoshuaWireBoardTest, SetTargetPositionSendsSetTargetFrame) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());
  auto channel = board.OpenChannel(0);
  ASSERT_TRUE(channel.ok());

  transport_->QueueResponse(MakeStatusResponse(JW1_CMD_SET_TARGET, 0, JW1_STATUS_OK));
  auto status = (*channel)->SetTarget(TargetMode::kPosition, 1234.0f);

  ASSERT_TRUE(status.ok()) << status;
  EXPECT_EQ(transport_->last_sent_[3], JW1_CMD_SET_TARGET);
  EXPECT_EQ(transport_->last_sent_[5], JW1_MODE_POSITION);
}

TEST_F(JoshuaWireBoardTest, SetTargetTorqueIsUnimplementedWithoutTouchingWire) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());
  auto channel = board.OpenChannel(0);
  ASSERT_TRUE(channel.ok());
  const int calls_before = transport_->send_calls_;

  EXPECT_EQ((*channel)->SetTarget(TargetMode::kTorque, 1.0f).code(),
            absl::StatusCode::kUnimplemented);
  EXPECT_EQ(transport_->send_calls_, calls_before);
}

TEST_F(JoshuaWireBoardTest, ReadFeedbackDecodesPositionAndVelocity) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());
  auto channel = board.OpenChannel(0);
  ASSERT_TRUE(channel.ok());

  transport_->QueueResponse(MakeFeedbackResponse(0, 4200.0f, -15.5f, 0));
  auto feedback = (*channel)->ReadFeedback();

  ASSERT_TRUE(feedback.ok()) << feedback.status();
  EXPECT_FLOAT_EQ(feedback->position, 4200.0f);
  EXPECT_FLOAT_EQ(feedback->velocity, -15.5f);
  EXPECT_EQ(feedback->fault_flags, 0);
}

TEST_F(JoshuaWireBoardTest, OpenChannelOutOfRangeIsNotFound) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());

  auto channel = board.OpenChannel(5);

  EXPECT_EQ(channel.status().code(), absl::StatusCode::kNotFound);
}

TEST_F(JoshuaWireBoardTest, TeardownDropsChannels) {
  QueueSuccessfulInit();
  FakeJoshuaWireBoard board;
  ASSERT_TRUE(board.Init(MakeBoardConfig()).ok());

  ASSERT_TRUE(board.Teardown().ok());

  EXPECT_EQ(board.OpenChannel(0).status().code(), absl::StatusCode::kFailedPrecondition);
}

}  // namespace
}  // namespace robot::board
