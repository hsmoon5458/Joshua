#include "robot/perception/position/position_sensor.h"

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "gtest/gtest.h"

namespace robot::perception {
namespace {

// Records every call, so a test can assert the sensor reads and does
// nothing else -- no board, no port, no hardware needed.
class RecordingChannel : public robot::board::BoardChannel {
 public:
  explicit RecordingChannel(robot::board::ChannelFeedback feedback) : feedback_(feedback) {}

  absl::Status Enable() override {
    ++commands_;
    return absl::OkStatus();
  }
  absl::Status Disable() override {
    ++commands_;
    return absl::OkStatus();
  }
  absl::Status SetTarget(robot::board::TargetMode, float) override {
    ++commands_;
    return absl::OkStatus();
  }
  absl::StatusOr<robot::board::ChannelFeedback> ReadFeedback() override {
    if (fail_) {
      return absl::UnavailableError("no response from channel");
    }
    ++reads_;
    return feedback_;
  }

  void FailNextReads() {
    fail_ = true;
  }
  int reads() const {
    return reads_;
  }
  int commands() const {
    return commands_;
  }

 private:
  robot::board::ChannelFeedback feedback_;
  bool fail_ = false;
  int reads_ = 0;
  int commands_ = 0;
};

robot::perception::Sensor MakeSensorConfig() {
  robot::perception::Sensor sensor;
  sensor.set_sensor_name("joint_1");
  sensor.set_id(1);
  sensor.set_sensor_type(robot::perception::SensorType::POSITION);
  sensor.set_board_name("arm_bus");
  sensor.set_channel(1);
  return sensor;
}

TEST(PositionSensorTest, PublishesThePositionTheChannelReports) {
  auto channel = std::make_shared<RecordingChannel>(
      robot::board::ChannelFeedback{.position = 2048.0f, .velocity = 12.0f});
  PositionSensor sensor(channel, MakeSensorConfig());

  ASSERT_TRUE(sensor.Init().ok());
  auto packet = sensor.GetData();
  ASSERT_TRUE(packet.ok()) << packet.status();
  EXPECT_EQ(packet->perception_id(), "joint_1");
  EXPECT_FLOAT_EQ(packet->position().position(), 2048.0f);
  EXPECT_FLOAT_EQ(packet->position().velocity(), 12.0f);
  EXPECT_GT(packet->timestamp_ns(), 0);
}

// A sensor observes; it must never command. The channel it holds can do
// both -- a channel is a channel -- so this is the guarantee, and it is
// worth asserting because perception publishers run in their own process
// while an actuator node drives the same bus.
TEST(PositionSensorTest, NeverCommandsTheChannel) {
  auto channel = std::make_shared<RecordingChannel>(robot::board::ChannelFeedback{});
  PositionSensor sensor(channel, MakeSensorConfig());

  ASSERT_TRUE(sensor.Init().ok());
  ASSERT_TRUE(sensor.GetData().ok());
  ASSERT_TRUE(sensor.GetData().ok());
  ASSERT_TRUE(sensor.Teardown().ok());

  EXPECT_EQ(channel->commands(), 0);
  EXPECT_EQ(channel->reads(), 2);
}

// Init must not read: the board already ran IDENTIFY, and a transient
// failure at startup would drop the sensor for the whole session.
TEST(PositionSensorTest, InitDoesNotTouchTheBus) {
  auto channel = std::make_shared<RecordingChannel>(robot::board::ChannelFeedback{});
  PositionSensor sensor(channel, MakeSensorConfig());

  ASSERT_TRUE(sensor.Init().ok());
  EXPECT_EQ(channel->reads(), 0);
}

// The board owns the port and shares it with anything else on the bus, so
// teardown must not close it.
TEST(PositionSensorTest, TeardownLeavesTheBoardAlone) {
  auto channel = std::make_shared<RecordingChannel>(robot::board::ChannelFeedback{});
  PositionSensor sensor(channel, MakeSensorConfig());

  EXPECT_TRUE(sensor.Teardown().ok());
  EXPECT_TRUE(sensor.GetData().ok());
}

TEST(PositionSensorTest, PropagatesReadFailures) {
  auto channel = std::make_shared<RecordingChannel>(robot::board::ChannelFeedback{});
  channel->FailNextReads();
  PositionSensor sensor(channel, MakeSensorConfig());

  const auto packet = sensor.GetData();
  ASSERT_FALSE(packet.ok());
  EXPECT_EQ(packet.status().code(), absl::StatusCode::kUnavailable);
}

}  // namespace
}  // namespace robot::perception
