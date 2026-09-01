#include <filesystem>
#include <string>
#include <vector>

#include "config/config_utils.h"
#include "gtest/gtest.h"
#include "robot/board/factory/sensor_channel_validation.h"

namespace {

namespace fs = std::filesystem;
constexpr auto kConfigPresetDirectory = "config/config_preset";

TEST(ConfigValidationTest, ValidateAllConfigPresets) {
  const std::string directory = kConfigPresetDirectory;

  if (!fs::exists(directory)) {
    FAIL() << "Config directory not found: " << directory;
  }

  int checked_files = 0;
  for (const auto& entry : fs::recursive_directory_iterator(directory)) {
    if (entry.path().extension() == ".pbtxt") {
      const std::string config_path = entry.path().string();
      auto result = config::config_util::LoadConfig(config_path);
      // EXPECT_TRUE continues execution even on failure
      EXPECT_TRUE(result.ok()) << "Failed to load config: " << config_path
                               << "\nError: " << result.status().message();
      checked_files++;
    }
  }
}

// Every configured sensor must declare a complete board or device path.
TEST(ConfigValidationTest, EverySensorResolves) {
  const std::string directory = kConfigPresetDirectory;
  ASSERT_TRUE(fs::exists(directory)) << "Config directory not found: " << directory;

  for (const auto& entry : fs::recursive_directory_iterator(directory)) {
    if (entry.path().extension() != ".pbtxt") {
      continue;
    }
    const std::string config_path = entry.path().string();
    auto config = config::config_util::LoadConfig(config_path);
    ASSERT_TRUE(config.ok()) << config_path << ": " << config.status().message();

    for (const auto& board : config->robot().boards()) {
      if (board.has_comm()) {
        EXPECT_NE(board.comm().transport_type(), robot::comm::TransportType::TRANSPORT_INVALID)
            << config_path << ": board '" << board.name() << "' has no transport_type.";
      }
    }

    for (const auto& single_perception : config->robot().perceptions().single_perceptions()) {
      const auto& sensor = single_perception.sensor();
      const std::string where = config_path + ": sensor '" + sensor.sensor_name() + "'";

      ASSERT_NE(sensor.sensor_type(), robot::perception::SensorType::SENSOR_INVALID)
          << where << " has no sensor_type.";
      if (sensor.has_comm()) {
        EXPECT_NE(sensor.comm().transport_type(), robot::comm::TransportType::TRANSPORT_INVALID)
            << where << " has no transport_type.";
      }

      const bool on_board = !sensor.board_name().empty();
      const bool has_device_config =
          sensor.sensor_config_case() != robot::perception::Sensor::SENSOR_CONFIG_NOT_SET;

      EXPECT_FALSE(on_board && (has_device_config || sensor.has_comm()))
          << where << " declares both a board and its own comm/device config.";
      ASSERT_TRUE(on_board || has_device_config)
          << where << " declares neither a board nor a device config.";

      if (!on_board) {
        continue;  // Device leg: the sensor_config oneof selects the driver.
      }

      const robot::board::Board* board = nullptr;
      for (const auto& candidate : config->robot().boards()) {
        if (candidate.name() == sensor.board_name()) {
          board = &candidate;
          break;
        }
      }
      ASSERT_NE(board, nullptr) << where << " references board '" << sensor.board_name()
                                << "' but no boards{} entry declares that name.";

      const robot::board::Channel* channel = nullptr;
      for (const auto& candidate : board->channels()) {
        if (candidate.index() == sensor.channel()) {
          channel = &candidate;
          break;
        }
      }
      ASSERT_NE(channel, nullptr) << where << " uses channel " << sensor.channel() << " but board '"
                                  << board->name() << "' does not declare it.";

      const auto status =
          robot::board::ValidateSensorChannel(sensor.sensor_type(), channel->signal());
      EXPECT_TRUE(status.ok()) << where << ": " << status.message();
    }
  }
}
}  // namespace
