#include "robot/board/feetech_bus/feetech_bus_board.h"

#include <memory>

#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "robot/board/feetech_bus/feetech_protocol.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"
#include "robot/comm/serial/fake_serial_transport.h"

namespace robot::board {
namespace {

using robot::comm::FakeSerialTransport;

// Builds a well-formed Feetech status/response packet with the given
// parameter bytes and error byte, matching the checksum arithmetic in
// feetech_protocol.cc. Test-only: production code never builds responses,
// only requests.
std::vector<uint8_t> MakeStatusResponse(uint8_t servo_id,
                                        uint8_t error,
                                        const std::vector<uint8_t>& params) {
  const uint8_t length = static_cast<uint8_t>(params.size() + 2);
  std::vector<uint8_t> frame_from_id = {servo_id, length, error};
  frame_from_id.insert(frame_from_id.end(), params.begin(), params.end());
  uint32_t sum = 0;
  for (uint8_t byte : frame_from_id) sum += byte;
  const uint8_t checksum = static_cast<uint8_t>(~sum & 0xFF);

  std::vector<uint8_t> response = {0xFF, 0xFF};
  response.insert(response.end(), frame_from_id.begin(), frame_from_id.end());
  response.push_back(checksum);
  return response;
}

robot::board::Board MakeArmBusBoard() {
  robot::board::Board board;
  board.set_name("arm_bus");
  board.set_board_type(robot::board::BoardType::FEETECH_BUS);
  auto* comm = board.mutable_comm();
  comm->set_comm_type(robot::comm::CommType::SERIAL);
  comm->set_transport_type(robot::comm::TransportType::MESSAGE);
  comm->mutable_serial_config()->set_port("/dev/ttyACM0");
  comm->mutable_serial_config()->set_baudrate(1000000);

  auto* channel = board.add_channels();
  channel->set_index(1);
  channel->set_drive(robot::board::DriveInterface::SERVO_BUS_UART);
  channel->mutable_servo_bus()->set_servo_id(5);
  channel->mutable_servo_bus()->set_move_time_ms(40);
  return board;
}

class FeetechBusBoardTest : public ::testing::Test {
 protected:
  void SetUp() override {
    transport_ = std::make_shared<FakeSerialTransport>();
    FeetechBusBoard::SetSerialTransportFactoryForTesting(
        [this](const robot::comm::Comm&)
            -> absl::StatusOr<std::shared_ptr<robot::comm::SerialTransport>> {
          return transport_;
        });
  }

  void TearDown() override {
    FeetechBusBoard::SetSerialTransportFactoryForTesting(nullptr);
  }

  // Every channel's Init() does a PING + read-model-number IDENTIFY, so
  // tests that expect Init to succeed must queue both acks first.
  void QueueIdentifyAcks(uint8_t servo_id) {
    transport_->QueueResponse(MakeStatusResponse(servo_id, /*error=*/0, {}));
    transport_->QueueResponse(MakeStatusResponse(servo_id, /*error=*/0, {0x0F, 0x03}));
  }

  std::shared_ptr<FakeSerialTransport> transport_;
};

TEST_F(FeetechBusBoardTest, InitIdentifiesEveryConfiguredServo) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;

  auto status = board.Init(MakeArmBusBoard());

  EXPECT_TRUE(status.ok()) << status;
  EXPECT_EQ(transport_->atomic_read_calls_, 2);
}

TEST_F(FeetechBusBoardTest, InitFailsWhenServoDoesNotRespond) {
  // No queued responses: FakeSerialTransport::AtomicRead returns zero bytes.
  FeetechBusBoard board;

  auto status = board.Init(MakeArmBusBoard());

  EXPECT_EQ(status.code(), absl::StatusCode::kUnavailable);
}

TEST_F(FeetechBusBoardTest, InitRejectsWrongBoardType) {
  auto config = MakeArmBusBoard();
  config.set_board_type(robot::board::BoardType::MOCK);
  FeetechBusBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(FeetechBusBoardTest, InitRejectsNonServoBusDrive) {
  auto config = MakeArmBusBoard();
  config.mutable_channels(0)->set_drive(robot::board::DriveInterface::STEP_DIR);
  FeetechBusBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(FeetechBusBoardTest, InitRejectsDuplicateServoId) {
  auto config = MakeArmBusBoard();
  auto* channel = config.add_channels();
  channel->set_index(2);
  channel->set_drive(robot::board::DriveInterface::SERVO_BUS_UART);
  channel->mutable_servo_bus()->set_servo_id(5);  // Same id as channel 1.
  FeetechBusBoard board;

  EXPECT_EQ(board.Init(config).code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(FeetechBusBoardTest, OpenChannelEnableWritesTorqueEnableRegister) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());

  auto channel = board.OpenChannel(1);
  ASSERT_TRUE(channel.ok()) << channel.status();
  ASSERT_TRUE((*channel)->Enable().ok());

  EXPECT_EQ(transport_->last_written_,
            (std::vector<uint8_t>{0xFF, 0xFF, 0x05, 0x04, 0x03, 0x28, 0x01, 0xCA}));
}

TEST_F(FeetechBusBoardTest, SetTargetPositionBundlesStagedSpeedAndConfiguredMoveTime) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());
  auto channel = board.OpenChannel(1);
  ASSERT_TRUE(channel.ok());

  ASSERT_TRUE((*channel)->SetTarget(TargetMode::kVelocity, 3000.0f).ok());
  ASSERT_TRUE((*channel)->SetTarget(TargetMode::kPosition, 2070.0f).ok());

  // position=2070 (0x0816), move_time_ms=40 (0x0028), speed=3000 (0x0BB8).
  auto written = transport_->last_written_;
  ASSERT_EQ(written.size(), 13u);
  EXPECT_EQ(written[6], 0x16);
  EXPECT_EQ(written[7], 0x08);
  EXPECT_EQ(written[8], 0x28);
  EXPECT_EQ(written[9], 0x00);
  EXPECT_EQ(written[10], 0xB8);
  EXPECT_EQ(written[11], 0x0B);
}

TEST_F(FeetechBusBoardTest, SetTargetTorqueIsUnimplemented) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());
  auto channel = board.OpenChannel(1);
  ASSERT_TRUE(channel.ok());

  EXPECT_EQ((*channel)->SetTarget(TargetMode::kTorque, 1.0f).code(),
            absl::StatusCode::kUnimplemented);
}

TEST_F(FeetechBusBoardTest, ReadFeedbackDecodesPresentPosition) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());
  auto channel = board.OpenChannel(1);
  ASSERT_TRUE(channel.ok());

  transport_->QueueResponse(MakeStatusResponse(5, /*error=*/0, {0x16, 0x08}));  // 2070.

  auto feedback = (*channel)->ReadFeedback();

  ASSERT_TRUE(feedback.ok()) << feedback.status();
  EXPECT_FLOAT_EQ(feedback->position, 2070.0f);
}

TEST_F(FeetechBusBoardTest, OpenChannelOutOfRangeIsNotFound) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());

  auto channel = board.OpenChannel(9);

  EXPECT_EQ(channel.status().code(), absl::StatusCode::kNotFound);
}

TEST_F(FeetechBusBoardTest, TeardownDropsChannels) {
  QueueIdentifyAcks(5);
  FeetechBusBoard board;
  ASSERT_TRUE(board.Init(MakeArmBusBoard()).ok());

  ASSERT_TRUE(board.Teardown().ok());

  EXPECT_EQ(board.OpenChannel(1).status().code(), absl::StatusCode::kFailedPrecondition);
}

}  // namespace
}  // namespace robot::board
