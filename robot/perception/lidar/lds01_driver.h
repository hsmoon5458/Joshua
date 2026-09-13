#pragma once
#include <glog/logging.h>

#include <atomic>
#include <memory>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "robot/comm/interfaces/byte_stream.h"
#include "robot/perception/interfaces/lidar_interface.h"
#include "robot/perception/proto/perception.pb.h"
#include "robot/perception/proto/perception_packet.pb.h"

namespace robot::perception {
class Lds01Driver : public LidarInterface {
 public:
  Lds01Driver(std::shared_ptr<robot::comm::ByteStream> stream,
              const robot::perception::Sensor& sensor_config);
  ~Lds01Driver() = default;

  absl::Status Init() override;
  std::string GetId() override;
  absl::StatusOr<robot::perception::PerceptionPacket> GetData() override;
  absl::Status Teardown() override;

 private:
  void reading_thread_func();

  std::shared_ptr<robot::comm::ByteStream> stream_;
  std::string id_;
  mutable robot::perception::PerceptionPacket reusable_packet_;
  std::thread receiving_thread_;
  std::atomic<bool> stop_receiving_;
};
}  // namespace robot::perception
