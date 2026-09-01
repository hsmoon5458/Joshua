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
#include "sensor_msgs/msg/image.hpp"

namespace {

bool ValidateCameraPublisher(const ros2::data_type::Ros2DataType ros2_data_type) {
  return ros2_data_type == ros2::data_type::IMAGE;
}

}  // namespace

class CameraPublisher : public rclcpp::Node {
 private:
  struct Camera {
    std::string topic;
    std::shared_ptr<robot::perception::PerceptionInterface> interface;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
  };

 public:
  CameraPublisher(const std::string& node_name, const int node_id, const config::Config& config)
      : Node(node_name) {
    for (const auto& single_perception : config.robot().perceptions().single_perceptions()) {
      if (single_perception.sensor().sensor_type() != robot::perception::SensorType::IMAGE ||
          static_cast<int>(single_perception.node().id()) != node_id) {
        continue;
      }

      const auto& sensor_proto = single_perception.sensor();
      const auto& qos_setting = single_perception.node().qos_setting();

      auto interface = robot::perception::PerceptionFactory::CreatePerception(
          single_perception, config.robot().boards());
      if (!interface.ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create perception interface for camera '%s': %s",
                     sensor_proto.sensor_name().c_str(),
                     std::string(interface.status().message()).c_str());
        continue;
      }

      auto shared_interface =
          std::shared_ptr<robot::perception::PerceptionInterface>(std::move(interface.value()));

      for (const auto& publisher : single_perception.node().publishers()) {
        if (!ValidateCameraPublisher(publisher.ros2_data_type())) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid publisher config for camera topic '%s' "
                       "(ros2_data_type=%d). Require IMAGE.",
                       publisher.topic().c_str(),
                       static_cast<int>(publisher.ros2_data_type()));
          continue;
        }

        const auto qos = ros2_utils::CreateQosSetting(qos_setting);
        cameras_.emplace_back(Camera{
            .topic = publisher.topic(),
            .interface = shared_interface,
            .publisher = this->create_publisher<sensor_msgs::msg::Image>(publisher.topic(), qos),
            .timer = this->create_wall_timer(
                std::chrono::milliseconds(1000 / publisher.publish_rate_hz()),
                [this]() { publish_camera_data(); })});
      }

      RCLCPP_INFO(this->get_logger(),
                  "Found camera '%s' in configuration for node_id %d. Publishing on %zu topics",
                  sensor_proto.sensor_name().c_str(),
                  node_id,
                  single_perception.node().publishers().size());
    }

    if (cameras_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "No camera found in configuration for node_id %d!", node_id);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Camera publisher node started with %zu cameras for node_id %d!",
                cameras_.size(),
                node_id);
  }

 private:
  void publish_camera_data() {
    if (cameras_.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Camera not initialized!");
      return;
    }

    try {
      for (const auto& camera : cameras_) {
        auto packet = camera.interface->GetData();

        if (!packet.ok()) {
          RCLCPP_WARN(
              this->get_logger(), "Failed to get data from camera '%s'!", camera.topic.c_str());
          continue;
        }

        const auto image_status = ros2_utils::RequirePerceptionImage(packet.value());
        if (!image_status.ok()) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to get image data from camera '%s': %s",
                      camera.topic.c_str(),
                      image_status.message().data());
          continue;
        }

        const auto& image_data = packet.value().image();

        if (image_data.width() <= 0 || image_data.height() <= 0 || image_data.channels() <= 0) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid image dimensions: %dx%d, channels=%d",
                       image_data.width(),
                       image_data.height(),
                       image_data.channels());
          continue;
        }

        auto image_msg = std::make_unique<sensor_msgs::msg::Image>();
        image_msg->header.stamp = this->now();
        image_msg->header.frame_id = "camera_frame";
        image_msg->height = image_data.height();
        image_msg->width = image_data.width();
        image_msg->encoding = "rgb8";
        image_msg->is_bigendian = false;
        image_msg->step = image_data.width() * 3;

        size_t data_size = image_data.width() * image_data.height() * 3;
        image_msg->data.resize(data_size);

        const uint8_t* bgr_data = reinterpret_cast<const uint8_t*>(image_data.data().data());
        for (size_t i = 0; i < data_size; i += 3) {
          image_msg->data[i] = bgr_data[i + 2];
          image_msg->data[i + 1] = bgr_data[i + 1];
          image_msg->data[i + 2] = bgr_data[i];
        }

        camera.publisher->publish(*image_msg);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing camera data: %s", e.what());
    }
  }

  std::vector<Camera> cameras_;
};

int main(int argc, char* argv[]) {
  return ros2_utils::RunNode<CameraPublisher>(argc, argv, "camera_publisher");
}
