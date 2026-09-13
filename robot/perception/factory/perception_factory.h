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
#include "robot/perception/encoder/sts3215_encoder.h"
#include "robot/perception/interfaces/perception_interface.h"
#include "robot/perception/lidar/lds01_driver.h"
#include "robot/perception/sensors/joint_position_sensor.h"
#include "utils/status_macros.h"

namespace robot::perception {
class PerceptionFactory {
 public:
  // Callers provide the configured boards used to resolve board-attached
  // sensors.
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreatePerception(
      const robot::perception::SinglePerception& single_perception,
      const google::protobuf::RepeatedPtrField<robot::board::Board>& boards) {
    if (single_perception.has_sensor()) {
      return CreateSensor(single_perception.sensor(), boards);
    }
    switch (single_perception.perception_type()) {
      case PerceptionType::CAMERA: {
        const auto& camera = single_perception.camera();
        return std::make_unique<CvCamera>(camera);
      }
      case PerceptionType::ENCODER: {
        const auto& encoder_config = single_perception.encoder();
        switch (encoder_config.encoder_type()) {
          case EncoderType::STS3215_ENCODER: {
            ABSL_ASSIGN_OR_RETURN(auto comm,
                                  robot::comm::CommFactory::CreateComm(encoder_config.comm()));
            ABSL_ASSIGN_OR_RETURN(
                auto transport, robot::comm::GetCommTransport<robot::comm::MessageTransport>(comm));
            auto encoder = std::make_unique<Sts3215Encoder>(transport, encoder_config);
            ABSL_RETURN_IF_ERROR(encoder->Init());
            return encoder;
          }
          default:
            return absl::InvalidArgumentError("Invalid encoder type.");
        }
      }
      case PerceptionType::LIDAR: {
        const auto& lidar_config = single_perception.lidar();
        switch (lidar_config.lidar_type()) {
          case LidarType::LDS01: {
            ABSL_ASSIGN_OR_RETURN(auto comm,
                                  robot::comm::CommFactory::CreateComm(lidar_config.comm()));
            ABSL_ASSIGN_OR_RETURN(auto stream,
                                  robot::comm::GetCommTransport<robot::comm::ByteStream>(comm));
            auto lidar = std::make_unique<Lds01Driver>(stream, lidar_config);
            ABSL_RETURN_IF_ERROR(lidar->Init());
            return lidar;
          }
          default:
            return absl::InvalidArgumentError("Invalid lidar type.");
        }
      }
      default:
        return absl::InvalidArgumentError("Invalid perception type.");
    }
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

    if (!on_board) {
      return absl::UnimplementedError(
          absl::StrCat(owner, ": single-stream devices have not moved to Sensor yet."));
    }
    return CreateBoardSensor(sensor, boards, owner);
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
      case robot::perception::SensorType::JOINT_POSITION:
        return std::make_unique<JointPositionSensor>(channel, sensor);
      default:
        return absl::UnimplementedError(
            absl::StrCat(owner,
                         ": sensor_type ",
                         robot::perception::SensorType_Name(sensor.sensor_type()),
                         " has no board-attached sensor driver yet."));
    }
  }

  PerceptionFactory() = default;
};
}  // namespace robot::perception
