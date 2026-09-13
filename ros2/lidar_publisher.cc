#include <cmath>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "config/proto/config.pb.h"
#include "rclcpp/rclcpp.hpp"
#include "robot/perception/factory/perception_factory.h"
#include "robot/perception/proto/perception_packet.pb.h"
#include "ros2/node_runner.h"
#include "ros2/proto/ros2_data_type.pb.h"
#include "ros2/utils/packet_parser.h"
#include "ros2/utils/qos_setting.h"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/point_cloud2_iterator.hpp"

namespace {

bool ValidateLidarPublisher(const ros2::data_type::Ros2DataType ros2_data_type) {
  return ros2_data_type == ros2::data_type::POINTCLOUD2;
}

}  // namespace

class LidarPublisher : public rclcpp::Node {
 private:
  struct Lidar {
    std::string topic;
    std::shared_ptr<robot::perception::PerceptionInterface> interface;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
    std::string frame_id;
  };

 public:
  LidarPublisher(const std::string& node_name, const int node_id, const config::Config& config)
      : Node(node_name) {
    for (const auto& single_perception : config.robot().perceptions().single_perceptions()) {
      if (single_perception.sensor().sensor_type() != robot::perception::SensorType::RANGE_SCAN ||
          static_cast<int>(single_perception.node().id()) != node_id) {
        continue;
      }

      const auto& sensor_proto = single_perception.sensor();
      const auto& qos_setting = single_perception.node().qos_setting();

      auto interface = robot::perception::PerceptionFactory::CreatePerception(
          single_perception, config.robot().boards());
      if (!interface.ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create perception interface for lidar '%s': %s",
                     sensor_proto.sensor_name().c_str(),
                     std::string(interface.status().message()).c_str());
        continue;
      }

      auto shared_interface =
          std::shared_ptr<robot::perception::PerceptionInterface>(std::move(interface.value()));

      for (const auto& publisher : single_perception.node().publishers()) {
        if (!ValidateLidarPublisher(publisher.ros2_data_type())) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid publisher config for lidar topic '%s' "
                       "(ros2_data_type=%d). Require POINTCLOUD2.",
                       publisher.topic().c_str(),
                       static_cast<int>(publisher.ros2_data_type()));
          continue;
        }

        const auto qos = ros2_utils::CreateQosSetting(qos_setting);
        lidars_.emplace_back(
            Lidar{.topic = publisher.topic(),
                  .interface = shared_interface,
                  .publisher =
                      this->create_publisher<sensor_msgs::msg::PointCloud2>(publisher.topic(), qos),
                  .timer = this->create_wall_timer(
                      std::chrono::milliseconds(1000 / publisher.publish_rate_hz()),
                      [this]() { publish_lidar_data(); }),
                  .frame_id = sensor_proto.sensor_name().empty() ? "lidar_frame"
                                                                 : sensor_proto.sensor_name()});
      }

      RCLCPP_INFO(this->get_logger(),
                  "Found lidar '%s' in configuration for node_id %d. Publishing on %zu topics",
                  sensor_proto.sensor_name().c_str(),
                  node_id,
                  single_perception.node().publishers().size());
    }

    if (lidars_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No lidars found in configuration for node_id %d!", node_id);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Lidar publisher node started with %zu lidars for node_id %d!",
                lidars_.size(),
                node_id);
  }

 private:
  void publish_lidar_data() {
    if (lidars_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No lidars initialized, skipping publish cycle.");
      return;
    }

    try {
      for (auto& lidar : lidars_) {
        auto packet = lidar.interface->GetData();

        if (!packet.ok()) {
          RCLCPP_WARN(
              this->get_logger(), "Failed to get data from lidar '%s'!", lidar.topic.c_str());
          continue;
        }

        const auto cloud_status = ros2_utils::RequirePerceptionPointCloud(packet.value());
        if (!cloud_status.ok()) {
          RCLCPP_WARN(this->get_logger(),
                      "LiDAR '%s' packet has no point cloud: %s",
                      lidar.topic.c_str(),
                      cloud_status.message().data());
          continue;
        }

        const auto& cloud = packet.value().point_cloud();
        const int num_points = cloud.x_size();
        if (num_points == 0) {
          RCLCPP_WARN(this->get_logger(), "Empty point cloud from '%s'!", lidar.topic.c_str());
          continue;
        }

        sensor_msgs::msg::PointCloud2 cloud_msg;
        cloud_msg.header.stamp = this->get_clock()->now();
        cloud_msg.header.frame_id = cloud.frame_id().empty() ? lidar.frame_id : cloud.frame_id();
        cloud_msg.height = cloud.height() > 0 ? cloud.height() : 1;
        cloud_msg.width = cloud.width() > 0 ? cloud.width() : static_cast<uint32_t>(num_points);
        cloud_msg.is_bigendian = false;
        cloud_msg.is_dense = cloud.is_dense();

        sensor_msgs::PointCloud2Modifier modifier(cloud_msg);
        modifier.setPointCloud2Fields(4,
                                      "x",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "y",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "z",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32,
                                      "intensity",
                                      1,
                                      sensor_msgs::msg::PointField::FLOAT32);
        modifier.resize(num_points);

        sensor_msgs::PointCloud2Iterator<float> iter_x(cloud_msg, "x");
        sensor_msgs::PointCloud2Iterator<float> iter_y(cloud_msg, "y");
        sensor_msgs::PointCloud2Iterator<float> iter_z(cloud_msg, "z");
        sensor_msgs::PointCloud2Iterator<float> iter_i(cloud_msg, "intensity");

        const int nx = cloud.x_size();
        const int ny = cloud.y_size();
        const int nz = cloud.z_size();
        const int ni = cloud.intensity_size();
        const int n = std::min({nx, ny, nz, std::max(1, ni)});
        for (int i = 0; i < n; ++i, ++iter_x, ++iter_y, ++iter_z, ++iter_i) {
          *iter_x = cloud.x(i);
          *iter_y = cloud.y(i);
          *iter_z = cloud.z(i);
          *iter_i = (i < ni) ? cloud.intensity(i) : 0.0f;
        }

        lidar.publisher->publish(cloud_msg);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing lidar data: %s", e.what());
    }
  }

  std::vector<Lidar> lidars_;
};

int main(int argc, char* argv[]) {
  return ros2_utils::RunNode<LidarPublisher>(argc, argv, "lidar_publisher");
}
