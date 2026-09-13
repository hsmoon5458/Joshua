#include "robot/board/factory/sensor_channel_validation.h"

#include <initializer_list>
#include <string>
#include <vector>

#include "absl/strings/str_cat.h"
#include "absl/strings/str_join.h"

namespace robot::board {

namespace {

// What each signal leg can measure. Adding a signal means adding one row;
// adding a sensor meaning means listing it under the legs that can produce
// it. Neither requires touching a sensor driver.
std::vector<robot::perception::SensorType> ProducibleBy(robot::board::SignalInterface signal) {
  switch (signal) {
    case robot::board::SignalInterface::SERVO_BUS_REGISTER:
    case robot::board::SignalInterface::QUADRATURE:
    case robot::board::SignalInterface::PDO_SLOT:
      return {robot::perception::SensorType::POSITION};
    case robot::board::SignalInterface::ANALOG_ADC:
      return {robot::perception::SensorType::FORCE_TORQUE};
    case robot::board::SignalInterface::SIGNAL_INVALID:
    default:
      return {};
  }
}

std::string NameList(const std::vector<robot::perception::SensorType>& types) {
  std::vector<std::string> names;
  names.reserve(types.size());
  for (const auto type : types) {
    names.push_back(robot::perception::SensorType_Name(type));
  }
  return absl::StrJoin(names, ", ");
}

}  // namespace

absl::Status ValidateSensorChannel(robot::perception::SensorType sensor_type,
                                   robot::board::SignalInterface signal) {
  if (sensor_type == robot::perception::SensorType::SENSOR_INVALID) {
    return absl::InvalidArgumentError("Sensor has an invalid sensor_type.");
  }
  if (signal == robot::board::SignalInterface::SIGNAL_INVALID) {
    return absl::InvalidArgumentError(
        "The channel declares no signal leg; set Channel.signal on any channel a sensor "
        "binds to.");
  }

  const auto producible = ProducibleBy(signal);
  for (const auto type : producible) {
    if (type == sensor_type) {
      return absl::OkStatus();
    }
  }
  return absl::InvalidArgumentError(
      absl::StrCat("A ",
                   robot::board::SignalInterface_Name(signal),
                   " channel cannot produce ",
                   robot::perception::SensorType_Name(sensor_type),
                   "; it measures ",
                   producible.empty() ? std::string("nothing") : NameList(producible),
                   "."));
}

}  // namespace robot::board
