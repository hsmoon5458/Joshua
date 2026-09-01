#include "robot/perception/factory/perception_factory.h"

#include <string>

#include "absl/status/status.h"
#include "config/proto/robot.pb.h"
#include "gtest/gtest.h"

namespace robot::perception {
namespace {

// Board leg: POSITION on a MOCK board's SERVO_BUS_REGISTER channel, so
// the resolution flow runs with no hardware -- the perception twin of
// ActionFactoryBoardPathTest.
robot::perception::SinglePerception MakeBoardSensor() {
  robot::perception::SinglePerception single_perception;
  auto* sensor = single_perception.mutable_sensor();
  sensor->set_sensor_name("joint_1");
  sensor->set_id(1);
  sensor->set_sensor_type(robot::perception::SensorType::POSITION);
  sensor->set_board_name("mock_bus_1");
  sensor->set_channel(1);
  sensor->set_operational_lower_limit(1147.0f);
  sensor->set_operational_upper_limit(3154.0f);
  return single_perception;
}

config::Robot MakeRobotWithMockServoBoard(
    robot::board::SignalInterface signal = robot::board::SignalInterface::SERVO_BUS_REGISTER) {
  config::Robot robot_config;
  auto* board = robot_config.add_boards();
  board->set_name("mock_bus_1");
  board->set_board_type(robot::board::BoardType::MOCK);
  auto* channel = board->add_channels();
  channel->set_index(1);
  channel->set_drive(robot::board::DriveInterface::SERVO_BUS_UART);
  channel->set_signal(signal);
  channel->mutable_servo_bus()->set_servo_id(1);
  return robot_config;
}

class PerceptionFactoryTest : public ::testing::Test {
 protected:
  void TearDown() override {
    robot::board::BoardFactory::ResetForTesting();
  }
};

TEST_F(PerceptionFactoryTest, CreatesPositionSensorOverMockBoardChannel) {
  auto robot_config = MakeRobotWithMockServoBoard();

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(MakeBoardSensor(),
                                                                          robot_config.boards());

  ASSERT_TRUE(sensor_or.ok()) << sensor_or.status();
  EXPECT_EQ((*sensor_or)->GetId(), "joint_1");
  EXPECT_TRUE((*sensor_or)->GetData().ok());
}

// The same sensor class, unchanged, over a different signal leg. This is the
// claim the SensorType/SignalInterface split makes: a new way of acquiring a
// joint angle needs no new sensor class.
TEST_F(PerceptionFactoryTest, SameSensorClassServesADifferentSignalLeg) {
  auto robot_config = MakeRobotWithMockServoBoard(robot::board::SignalInterface::QUADRATURE);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(MakeBoardSensor(),
                                                                          robot_config.boards());

  ASSERT_TRUE(sensor_or.ok()) << sensor_or.status();
  EXPECT_TRUE((*sensor_or)->GetData().ok());
}

TEST_F(PerceptionFactoryTest, SensorSharesTheBoardInstanceWithOtherChannels) {
  auto robot_config = MakeRobotWithMockServoBoard();

  auto first = robot::board::BoardFactory::GetOrCreate(robot_config.boards(0));
  ASSERT_TRUE(first.ok()) << first.status();

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(MakeBoardSensor(),
                                                                          robot_config.boards());
  ASSERT_TRUE(sensor_or.ok()) << sensor_or.status();

  auto second = robot::board::BoardFactory::GetOrCreate(robot_config.boards(0));
  ASSERT_TRUE(second.ok()) << second.status();
  EXPECT_EQ(first->get(), second->get());
}

TEST_F(PerceptionFactoryTest, RejectsSensorDeclaringBothLegs) {
  auto robot_config = MakeRobotWithMockServoBoard();
  auto single_perception = MakeBoardSensor();
  single_perception.mutable_sensor()->mutable_comm()->set_comm_type(robot::comm::CommType::SERIAL);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(sensor_or.status().message().find("both legs"), std::string::npos)
      << sensor_or.status().message();
}

TEST_F(PerceptionFactoryTest, RejectsSensorDeclaringNeitherLeg) {
  auto robot_config = MakeRobotWithMockServoBoard();
  auto single_perception = MakeBoardSensor();
  single_perception.mutable_sensor()->clear_board_name();

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(sensor_or.status().message().find("neither leg"), std::string::npos)
      << sensor_or.status().message();
}

TEST_F(PerceptionFactoryTest, RejectsUnknownBoardName) {
  auto robot_config = MakeRobotWithMockServoBoard();
  auto single_perception = MakeBoardSensor();
  single_perception.mutable_sensor()->set_board_name("no_such_board");

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kNotFound);
}

TEST_F(PerceptionFactoryTest, RejectsUndeclaredChannel) {
  auto robot_config = MakeRobotWithMockServoBoard();
  auto single_perception = MakeBoardSensor();
  single_perception.mutable_sensor()->set_channel(9);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kNotFound);
}

TEST_F(PerceptionFactoryTest, RejectsChannelWithNoSignalLeg) {
  auto robot_config = MakeRobotWithMockServoBoard(robot::board::SignalInterface::SIGNAL_INVALID);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(MakeBoardSensor(),
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(PerceptionFactoryTest, RejectsMeaningTheSignalLegCannotProduce) {
  auto robot_config = MakeRobotWithMockServoBoard();
  auto single_perception = MakeBoardSensor();
  single_perception.mutable_sensor()->set_sensor_type(robot::perception::SensorType::IMAGE);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
}

// Device leg: a sensor_config whose meaning disagrees with sensor_type is a
// config error, not a silently mismatched driver.
TEST_F(PerceptionFactoryTest, RejectsDeviceConfigThatContradictsSensorType) {
  config::Robot robot_config;
  robot::perception::SinglePerception single_perception;
  auto* sensor = single_perception.mutable_sensor();
  sensor->set_sensor_name("webcam");
  sensor->set_sensor_type(robot::perception::SensorType::RANGE_SCAN);
  sensor->mutable_opencv_config()->set_id(0);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(sensor_or.status().message().find("opencv_config"), std::string::npos)
      << sensor_or.status().message();
}

// A byte-stream sensor on a datagram link fails in the composition factory,
// not in the driver -- capability compatibility is checked before construction.
TEST_F(PerceptionFactoryTest, RejectsDatagramCommForAByteStreamSensor) {
  config::Robot robot_config;
  robot::perception::SinglePerception single_perception;
  auto* sensor = single_perception.mutable_sensor();
  sensor->set_sensor_name("lidar_1");
  sensor->set_sensor_type(robot::perception::SensorType::RANGE_SCAN);
  sensor->mutable_lds01_config();
  sensor->mutable_comm()->set_comm_type(robot::comm::CommType::ETHERNET_UDP);
  sensor->mutable_comm()->set_transport_type(robot::comm::TransportType::BYTE_STREAM);

  auto sensor_or = robot::perception::PerceptionFactory::CreatePerception(single_perception,
                                                                          robot_config.boards());

  ASSERT_FALSE(sensor_or.ok());
  EXPECT_EQ(sensor_or.status().code(), absl::StatusCode::kInvalidArgument);
}

}  // namespace
}  // namespace robot::perception
