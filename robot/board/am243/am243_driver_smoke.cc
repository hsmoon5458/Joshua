// Board-level serial smoke test for an AM243 running joshua_wire_v1.
// This opens the configured serial port and can move physical hardware;
// inspect the pin configuration and wiring before running it.
//
// Usage:
//   bazel run //robot/board/am243:am243_driver_smoke -- [port]
#include <glog/logging.h>

#include <chrono>
#include <cstdio>
#include <string>
#include <thread>

#include "robot/board/am243/am243_board.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyACM0";

  robot::board::Board config;
  config.set_name("am243_stepper_bus");
  config.set_board_type(robot::board::BoardType::AM243);
  auto* comm = config.mutable_comm();
  comm->set_comm_type(robot::comm::CommType::SERIAL);
  comm->set_transport_type(robot::comm::TransportType::MESSAGE);
  comm->mutable_serial_config()->set_port(port);
  comm->mutable_serial_config()->set_baudrate(115200);
  config.mutable_firmware()->set_min_proto_version(1);
  auto* channel = config.add_channels();
  channel->set_index(0);
  channel->set_drive(robot::board::DriveInterface::STEP_DIR);
  channel->mutable_step_dir()->set_max_pulse_rate_hz(4000);
  channel->mutable_step_dir()->set_enable_active_low(true);
  channel->mutable_step_dir()->set_step_pin(2);
  channel->mutable_step_dir()->set_dir_pin(3);
  channel->mutable_step_dir()->set_enable_pin(4);

  robot::board::Am243Board board;
  auto status = board.Init(config);
  std::printf("Init: %s\n", status.ToString().c_str());
  if (!status.ok()) return 1;

  auto channel_or = board.OpenChannel(0);
  if (!channel_or.ok()) {
    std::printf("OpenChannel: %s\n", channel_or.status().ToString().c_str());
    return 1;
  }
  auto channel_handle = *channel_or;
  status = channel_handle->Enable();
  std::printf("Enable: %s\n", status.ToString().c_str());
  if (!status.ok()) return 1;

  for (int i = 0; i < 5; ++i) {
    const float target = (i % 2 == 0) ? 500.0f : -500.0f;
    status = channel_handle->SetTarget(robot::board::TargetMode::kPosition, target);
    std::printf("SetTarget[%d] target=%.1f: %s\n", i, target, status.ToString().c_str());
    if (!status.ok()) return 1;
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }
  return 0;
}
