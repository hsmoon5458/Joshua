#pragma once

#include <glog/logging.h>

#include <cstdint>
#include <mutex>
#include <opencv2/videoio.hpp>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/perception/interfaces/camera_interface.h"
#include "robot/perception/proto/perception.pb.h"
#include "robot/perception/proto/perception_packet.pb.h"

namespace robot::perception {

// Image sensor backed by an OpenCV capture device.
class CvCamera : public CameraInterface {
 public:
  explicit CvCamera(const robot::perception::Sensor& sensor_config);
  ~CvCamera() override;

  absl::Status Init() override;
  std::string GetId() override;
  absl::StatusOr<robot::perception::PerceptionPacket> GetData() override;
  absl::Status Teardown() override;

 private:
  const uint8_t MAX_CAMERA_OPEN_TRIES_ = 3;
  cv::VideoCapture cap_;
  mutable std::mutex cap_mutex_;
  std::string id_;
  uint64_t camera_id_;
  robot::perception::OpenCvConfig opencv_config_;
  mutable robot::perception::PerceptionPacket reusable_packet_;

  absl::Status OpenCameraLocked();
  absl::Status TeardownLocked();
};

}  // namespace robot::perception
