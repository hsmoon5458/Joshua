#pragma once

#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "config/proto/robot.pb.h"
#include "robot/comm/factory/comm_factory.h"
#include "robot/perception/camera/cv_camera.h"
#include "robot/perception/encoder/sts3215_encoder.h"
#include "robot/perception/interfaces/perception_interface.h"
#include "robot/perception/lidar/lds01_driver.h"
#include "utils/status_macros.h"

namespace robot::perception {
class PerceptionFactory {
 public:
  static absl::StatusOr<std::unique_ptr<robot::perception::PerceptionInterface>> CreatePerception(
      const robot::perception::SinglePerception& single_perception) {
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
  PerceptionFactory() = default;
};
}  // namespace robot::perception
