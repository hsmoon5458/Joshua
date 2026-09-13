#include <chrono>
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
#include "std_msgs/msg/float32.hpp"

namespace {

// The display name, whichever shape the entry uses.
std::string DisplayName(const robot::perception::SinglePerception& single_perception) {
  return single_perception.has_sensor() ? single_perception.sensor().sensor_name()
                                        : single_perception.encoder().encoder_name();
}

// True for either shape while presets migrate: the new Sensor message
// names the reading's meaning, the old one named a device family.
bool Matches(const robot::perception::SinglePerception& single_perception) {
  if (single_perception.has_sensor()) {
    return single_perception.sensor().sensor_type() ==
           robot::perception::SensorType::JOINT_POSITION;
  }
  return single_perception.perception_type() == robot::perception::PerceptionType::ENCODER;
}

bool ValidateEncoderPublisher(const ros2::data_type::Ros2DataType ros2_data_type) {
  return ros2_data_type == ros2::data_type::FLOAT32;
}

}  // namespace

class EncoderPublisher : public rclcpp::Node {
 private:
  struct Encoder {
    std::string topic;
    std::shared_ptr<robot::perception::PerceptionInterface> interface;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher;
    rclcpp::TimerBase::SharedPtr timer;
  };

 public:
  EncoderPublisher(const std::string& node_name, const int node_id, const config::Config& config)
      : Node(node_name) {
    for (const auto& single_perception : config.robot().perceptions().single_perceptions()) {
      if (!Matches(single_perception) ||
          static_cast<int>(single_perception.node().id()) != node_id) {
        continue;
      }

      const std::string sensor_name = DisplayName(single_perception);
      const auto& qos_setting = single_perception.node().qos_setting();

      auto interface = robot::perception::PerceptionFactory::CreatePerception(
          single_perception, config.robot().boards());
      if (!interface.ok()) {
        RCLCPP_ERROR(this->get_logger(),
                     "Failed to create perception interface for encoder '%s'. Check hardware "
                     "connection or permissions.",
                     sensor_name.c_str());
        continue;
      }

      auto shared_interface =
          std::shared_ptr<robot::perception::PerceptionInterface>(std::move(interface.value()));

      for (const auto& publisher : single_perception.node().publishers()) {
        if (!ValidateEncoderPublisher(publisher.ros2_data_type())) {
          RCLCPP_ERROR(this->get_logger(),
                       "Invalid publisher config for encoder topic '%s' "
                       "(ros2_data_type=%d). Require FLOAT32.",
                       publisher.topic().c_str(),
                       static_cast<int>(publisher.ros2_data_type()));
          continue;
        }

        encoders_.emplace_back(
            Encoder{.topic = publisher.topic(),
                    .interface = shared_interface,
                    .publisher = this->create_publisher<std_msgs::msg::Float32>(
                        publisher.topic(), ros2_utils::CreateQosSetting(qos_setting)),
                    .timer = this->create_wall_timer(
                        std::chrono::milliseconds(1000 / publisher.publish_rate_hz()),
                        [this]() { publish_encoder_data(); })});
      }

      RCLCPP_INFO(this->get_logger(),
                  "Found encoder '%s' in configuration for node_id %d. Publishing on %zu topics.",
                  sensor_name.c_str(),
                  node_id,
                  single_perception.node().publishers().size());
    }

    if (encoders_.empty()) {
      RCLCPP_ERROR(
          this->get_logger(), "No encoders found in configuration for node_id %d!", node_id);
      return;
    }

    RCLCPP_INFO(this->get_logger(),
                "Encoder publisher node started with %zu encoders for node_id %d!",
                encoders_.size(),
                node_id);
  }

 private:
  void publish_encoder_data() {
    if (encoders_.empty()) {
      RCLCPP_WARN(this->get_logger(), "No encoders initialized, skipping publish cycle.");
      return;
    }

    try {
      for (auto& encoder : encoders_) {
        auto packet = encoder.interface->GetData();

        if (!packet.ok()) {
          RCLCPP_WARN(
              this->get_logger(), "Failed to get data from encoder '%s'!", encoder.topic.c_str());
          continue;
        }

        auto position = ros2_utils::RequirePerceptionPosition(packet.value());
        if (!position.ok()) {
          RCLCPP_WARN(this->get_logger(),
                      "Failed to get position data from encoder '%s'!",
                      encoder.topic.c_str());
          continue;
        }

        std_msgs::msg::Float32 message;
        message.data = position.value();
        encoder.publisher->publish(message);
      }
    } catch (const std::exception& e) {
      RCLCPP_ERROR(this->get_logger(), "Error publishing encoder data: %s", e.what());
    }
  }

  std::vector<Encoder> encoders_;
};

int main(int argc, char* argv[]) {
  return ros2_utils::RunNode<EncoderPublisher>(argc, argv, "encoder_publisher");
}
