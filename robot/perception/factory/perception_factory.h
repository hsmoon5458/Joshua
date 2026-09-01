#pragma once

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "absl/strings/str_cat.h"
#include "config/proto/robot.pb.h"
#include "google/protobuf/repeated_ptr_field.h"
#include "robot/board/factory/board_factory.h"
#include "robot/board/factory/board_resolver.h"
#include "robot/board/factory/sensor_channel_validation.h"
#include "robot/comm/factory/comm_factory.h"
#include "robot/perception/camera/cv_camera.h"
#include "robot/perception/interfaces/perception_interface.h"
#include "robot/perception/lidar/lds01_driver.h"
#include "robot/perception/position/position_sensor.h"
#include "utils/status_macros.h"

namespace robot::perception {

// Creates sensors over either a board channel or a compatible transport
// capability, as selected by configuration.
class PerceptionFactory {
 public:
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreatePerception(
      const robot::perception::SinglePerception& single_perception,
      const google::protobuf::RepeatedPtrField<robot::board::Board>& boards) {
    return CreateSensor(single_perception.sensor(), boards);
  }

  ~PerceptionFactory() = default;
  PerceptionFactory(const PerceptionFactory&) = delete;
  PerceptionFactory& operator=(const PerceptionFactory&) = delete;
  PerceptionFactory(PerceptionFactory&&) = default;
  PerceptionFactory& operator=(PerceptionFactory&&) = default;

 private:
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreateSensor(
      const robot::perception::Sensor& sensor,
      const google::protobuf::RepeatedPtrField<robot::board::Board>& boards) {
    const std::string owner = absl::StrCat("Sensor '", sensor.sensor_name(), "'");

    if (sensor.sensor_type() == robot::perception::SensorType::SENSOR_INVALID) {
      return absl::InvalidArgumentError(absl::StrCat(owner, " has no sensor_type."));
    }

    const bool on_board = !sensor.board_name().empty();
    const bool has_device_config =
        sensor.sensor_config_case() != robot::perception::Sensor::SENSOR_CONFIG_NOT_SET;
    if (on_board && (has_device_config || sensor.has_comm())) {
      return absl::InvalidArgumentError(
          absl::StrCat(owner,
                       " declares both legs: it names a board and also carries its own comm or "
                       "device config. A sensor reaches hardware over exactly one."));
    }
    if (!on_board && !has_device_config) {
      return absl::InvalidArgumentError(
          absl::StrCat(owner,
                       " declares neither leg: set board_name + channel for a sensor on a board, "
                       "or a device config (opencv_config, lds01_config) for a single-stream "
                       "device."));
    }

    return on_board ? CreateBoardSensor(sensor, boards, owner) : CreateDeviceSensor(sensor, owner);
  }

  // Resolves and validates a board-attached sensor channel.
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreateBoardSensor(
      const robot::perception::Sensor& sensor,
      const google::protobuf::RepeatedPtrField<robot::board::Board>& boards,
      const std::string& owner) {
    ABSL_ASSIGN_OR_RETURN(
        auto resolved,
        robot::board::ResolveChannelConfig(boards, owner, sensor.board_name(), sensor.channel()));
    ABSL_RETURN_IF_ERROR(
        robot::board::ValidateSensorChannel(sensor.sensor_type(), resolved.channel->signal()));

    ABSL_ASSIGN_OR_RETURN(auto board, robot::board::BoardFactory::GetOrCreate(*resolved.board));
    ABSL_ASSIGN_OR_RETURN(auto channel, board->OpenChannel(sensor.channel()));

    switch (sensor.sensor_type()) {
      case robot::perception::SensorType::POSITION:
        return std::make_unique<PositionSensor>(channel, sensor);
      default:
        return absl::UnimplementedError(
            absl::StrCat(owner,
                         ": sensor_type ",
                         robot::perception::SensorType_Name(sensor.sensor_type()),
                         " has no board-attached sensor driver yet."));
    }
  }

  // Creates a sensor from its device-specific configuration.
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreateDeviceSensor(
      const robot::perception::Sensor& sensor, const std::string& owner) {
    switch (sensor.sensor_config_case()) {
      case robot::perception::Sensor::kOpencvConfig: {
        ABSL_RETURN_IF_ERROR(RequireSensorType(
            sensor, robot::perception::SensorType::IMAGE, owner, "opencv_config"));
        return std::make_unique<CvCamera>(sensor);
      }
      case robot::perception::Sensor::kLds01Config: {
        ABSL_RETURN_IF_ERROR(RequireSensorType(
            sensor, robot::perception::SensorType::RANGE_SCAN, owner, "lds01_config"));
        ABSL_ASSIGN_OR_RETURN(auto comm, robot::comm::CommFactory::CreateComm(sensor.comm()));
        ABSL_ASSIGN_OR_RETURN(auto stream,
                              robot::comm::GetCommTransport<robot::comm::ByteStream>(comm));
        auto lidar = std::make_unique<Lds01Driver>(stream, sensor);
        ABSL_RETURN_IF_ERROR(lidar->Init());
        return lidar;
      }
      case robot::perception::Sensor::SENSOR_CONFIG_NOT_SET:
      default:
        return absl::InvalidArgumentError(absl::StrCat(owner, " has no device config."));
    }
  }

  static absl::Status RequireSensorType(const robot::perception::Sensor& sensor,
                                        robot::perception::SensorType expected,
                                        const std::string& owner,
                                        const std::string& config_name) {
    if (sensor.sensor_type() == expected) {
      return absl::OkStatus();
    }
    return absl::InvalidArgumentError(
        absl::StrCat(owner,
                     " sets ",
                     config_name,
                     ", which produces ",
                     robot::perception::SensorType_Name(expected),
                     ", but declares sensor_type ",
                     robot::perception::SensorType_Name(sensor.sensor_type()),
                     "."));
  }

  PerceptionFactory() = default;
};
}  // namespace robot::perception
