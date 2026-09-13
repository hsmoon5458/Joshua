#include "robot/perception/lidar/lds01_driver.h"

#include <glog/logging.h>

#include <chrono>
#include <cmath>
#include <utility>

#include "utils/status_macros.h"

namespace robot::perception {

Lds01Driver::Lds01Driver(std::shared_ptr<robot::comm::ByteStream> stream,
                         const robot::perception::Lidar& lidar_config)
    : stream_(std::move(stream)) {
  id_ = std::to_string(lidar_config.id());
}

absl::Status Lds01Driver::Init() {
  ABSL_RETURN_IF_ERROR(stream_->Open());

  // Send start motor command (required for LDS-01)
  std::vector<uint8_t> start_cmd = {'b'};
  ABSL_RETURN_IF_ERROR(stream_->Write(start_cmd));

  stop_receiving_.store(false);
  return absl::OkStatus();
}

absl::Status Lds01Driver::Teardown() {
  stop_receiving_.store(true);

  // Send stop motor command
  std::vector<uint8_t> stop_cmd = {'e'};
  ABSL_RETURN_IF_ERROR(stream_->Write(stop_cmd));

  return absl::OkStatus();
}

std::string Lds01Driver::GetId() {
  auto id = "lds01_driver_" + id_;
  return id;
}

absl::StatusOr<robot::perception::PerceptionPacket> Lds01Driver::GetData() {
  uint8_t start_count = 0;
  bool got_scan = false;
  std::vector<uint8_t> raw_bytes(2520);
  uint8_t good_sets = 0;
  uint32_t motor_speed = 0;
  uint32_t rpms = 0;
  uint16_t index;

  LOG(INFO) << "Starting LiDAR data collection...";

  // Main loop for thread.
  while (!stop_receiving_.load(std::memory_order_acquire) && !got_scan) {
    // Wait until first data sync of frame: 0xFA, 0xA0
    ABSL_ASSIGN_OR_RETURN(auto result, stream_->Read(1));

    uint8_t byte = result[0];

    if (start_count == 0) {
      if (byte == 0xFA) {
        start_count = 1;
      }
    } else if (start_count == 1) {
      if (byte == 0xA0) {
        start_count = 0;
        // Start sequence found, read in the rest of the message via a single blocking read
        LOG(INFO) << "Found start sequence, reading full scan...";

        ABSL_ASSIGN_OR_RETURN(auto remain_result, stream_->Read(2518));

        // Reconstruct the full packet: 0xFA, 0xA0 + remaining data
        raw_bytes[0] = 0xFA;
        raw_bytes[1] = 0xA0;
        std::copy(remain_result.begin(), remain_result.end(), raw_bytes.begin() + 2);

        // Clear packet then set metadata before adding points
        reusable_packet_.Clear();
        reusable_packet_.set_perception_id(GetId());
        reusable_packet_.set_timestamp_ns(
            std::chrono::duration_cast<std::chrono::nanoseconds>(
                std::chrono::high_resolution_clock::now().time_since_epoch())
                .count());

        const size_t total_len = 2 + remain_result.size();

        // Prepare SoA vectors
        auto* cloud = reusable_packet_.mutable_point_cloud();
        cloud->clear_x();
        cloud->clear_y();
        cloud->clear_z();
        cloud->clear_intensity();
        cloud->clear_time_offset();
        cloud->set_height(1);
        cloud->set_is_dense(false);
        cloud->set_frame_id("lidar");

        for (uint16_t i = 0; i + 41 < total_len; i = i + 42) {
          if (raw_bytes[i] == 0xFA && raw_bytes[i + 1] == (0xA0 + i / 42)) {
            good_sets++;
            motor_speed += (raw_bytes[i + 3] << 8) + raw_bytes[i + 2];
            rpms = (raw_bytes[i + 3] << 8 | raw_bytes[i + 2]) / 10;

            for (uint16_t j = i + 4; j + 5 < total_len && j < i + 40; j = j + 6) {
              index = 6 * (i / 42) + (j - 4 - i) / 6;

              uint8_t byte0 = raw_bytes[j];
              uint8_t byte1 = raw_bytes[j + 1];
              uint8_t byte2 = raw_bytes[j + 2];
              uint8_t byte3 = raw_bytes[j + 3];

              uint16_t intensity = (byte1 << 8) + byte0;

              uint16_t range = (byte3 << 8) + byte2;

              // Convert to Cartesian assuming index is degrees
              const float r_m = static_cast<float>(range) * 0.001f;  // mm -> m
              const float theta_rad = static_cast<float>(index) * static_cast<float>(M_PI) / 180.0f;
              cloud->add_x(r_m * std::cos(theta_rad));
              cloud->add_y(r_m * std::sin(theta_rad));
              cloud->add_z(0.0f);
              cloud->add_intensity(static_cast<float>(intensity));
              // Optional time per point could be added here if known
            }
          }
        }

        // Set width to number of points
        cloud->set_width(static_cast<uint32_t>(cloud->x_size()));

        // Mark scan as complete only after successful accumulation and parsing
        got_scan = true;

      } else {
        start_count = 0;
      }
    }
  }

  LOG(INFO) << "Returning LiDAR packet with " << reusable_packet_.point_cloud().x_size()
            << " points";
  return reusable_packet_;
}
}  // namespace robot::perception
