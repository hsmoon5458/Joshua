// Board-level smoke test for Esp32Board, bypassing ActionFactory/ROS
// entirely (mirrors robot/board/teensy/teensy_driver_smoke.cc). Useful for
// isolating "is it the board/wire-protocol layer" from "is it something in
// the ActionFactory/ROS/actuator_subscriber stack" when bringing up a new
// ESP32 board.
//
// Usage:
//   bazel run //robot/board/esp32:esp32_driver_smoke -- [port]
//
// Runs Init() (IDENTIFY + CONFIGURE_CHANNEL), Enable()s channel 0, then
// alternates SetTarget(kPosition, ...) between +500/-500 native steps five
// times, printing every status. A working board/firmware/wiring chain
// prints "OK" for every line and the motor visibly moves; any failure
// (wrong firmware, bad wiring, unplugged board) surfaces as a non-OK
// status with an actionable message from JoshuaWireBoard/JoshuaWireChannel
// (robot/board/joshua_wire/), which Esp32Board is a thin subclass of.
#include <glog/logging.h>

#include <cstdio>
#include <thread>

#include "robot/board/esp32/esp32_board.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"

int main(int argc, char** argv) {
  google::InitGoogleLogging(argv[0]);
  // Most ESP32 dev boards enumerate via a USB-to-UART bridge (CP2102/CH340),
  // not native USB CDC, so this is /dev/ttyUSB0 rather than Teensy's
  // /dev/ttyACM0 — check `ls /dev/ttyUSB*` if this doesn't match.
  const std::string port = argc > 1 ? argv[1] : "/dev/ttyUSB0";

  robot::board::Board config;
  config.set_name("esp32_stepper_bus");
  config.set_board_type(robot::board::BoardType::ESP32);
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
  channel->mutable_step_dir()->set_step_pin(25);
  channel->mutable_step_dir()->set_dir_pin(26);
  channel->mutable_step_dir()->set_enable_pin(27);

  robot::board::Esp32Board board;
  auto init_status = board.Init(config);
  printf("Init: %s\n", init_status.ToString().c_str());
  if (!init_status.ok()) return 1;

  auto channel_or = board.OpenChannel(0);
  if (!channel_or.ok()) {
    printf("OpenChannel: %s\n", channel_or.status().ToString().c_str());
    return 1;
  }
  auto ch = *channel_or;

  auto enable_status = ch->Enable();
  printf("Enable: %s\n", enable_status.ToString().c_str());
  if (!enable_status.ok()) return 1;

  for (int i = 0; i < 5; i++) {
    float target = (i % 2 == 0) ? 500.0f : -500.0f;
    auto status = ch->SetTarget(robot::board::TargetMode::kPosition, target);
    printf("SetTarget[%d] target=%.1f: %s\n", i, target, status.ToString().c_str());
    std::this_thread::sleep_for(std::chrono::milliseconds(1500));
  }
  return 0;
}
