#include "robot/board/factory/sensor_channel_validation.h"

#include <string>

#include "absl/status/status.h"
#include "gtest/gtest.h"

namespace robot::board {
namespace {

TEST(SensorChannelValidationTest, PositionComesFromAnyPositionSignal) {
  for (const auto signal : {robot::board::SignalInterface::SERVO_BUS_REGISTER,
                            robot::board::SignalInterface::QUADRATURE,
                            robot::board::SignalInterface::PDO_SLOT}) {
    EXPECT_TRUE(ValidateSensorChannel(robot::perception::SensorType::POSITION, signal).ok())
        << robot::board::SignalInterface_Name(signal);
  }
}

TEST(SensorChannelValidationTest, RejectsSignalThatCannotProduceTheReading) {
  const auto status = ValidateSensorChannel(robot::perception::SensorType::POSITION,
                                            robot::board::SignalInterface::ANALOG_ADC);
  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(status.message().find("ANALOG_ADC"), std::string::npos) << status.message();
  EXPECT_NE(status.message().find("FORCE_TORQUE"), std::string::npos) << status.message();
}

TEST(SensorChannelValidationTest, RejectsChannelWithNoSignalLeg) {
  const auto status = ValidateSensorChannel(robot::perception::SensorType::POSITION,
                                            robot::board::SignalInterface::SIGNAL_INVALID);
  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(status.message().find("Channel.signal"), std::string::npos) << status.message();
}

TEST(SensorChannelValidationTest, RejectsInvalidSensorType) {
  EXPECT_EQ(ValidateSensorChannel(robot::perception::SensorType::SENSOR_INVALID,
                                  robot::board::SignalInterface::SERVO_BUS_REGISTER)
                .code(),
            absl::StatusCode::kInvalidArgument);
}

// A meaning no signal leg produces yet fails with a message that names what
// the leg does measure, rather than a bare "invalid".
TEST(SensorChannelValidationTest, RejectsUnsupportedMeaningWithAnActionableMessage) {
  const auto status = ValidateSensorChannel(robot::perception::SensorType::IMAGE,
                                            robot::board::SignalInterface::SERVO_BUS_REGISTER);
  EXPECT_EQ(status.code(), absl::StatusCode::kInvalidArgument);
  EXPECT_NE(status.message().find("POSITION"), std::string::npos) << status.message();
}

}  // namespace
}  // namespace robot::board
