#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "firmware/common/joshua_wire_v1.h"
#include "robot/board/frame/frame_transport.h"
#include "robot/board/interfaces/board_interface.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/proto/comm.pb.h"

namespace robot::board {

// Shared host-side implementation of the joshua_wire_v1 board contract:
// open a FrameTransport, run the
// IDENTIFY handshake (board_id, protocol version, per-channel drive all
// cross-checked against config), push CONFIGURE_CHANNEL for every channel,
// then dispatch ENABLE/DISABLE/SET_TARGET/GET_FEEDBACK per channel. Every
// Joshua-firmware MCU board (Am243Board, TeensyBoard, Esp32Board today;
// ArduinoBoard, ...) speaks the exact same wire protocol, so this class
// holds that entire orchestration once — a concrete board subclasses this
// and supplies only the handful of facts that actually differ per board:
// which BoardType/jw1_board_id_t it is, and (if it ever isn't a plain
// serial link) how its comm config is validated and its production
// transport is built.
//
// What is NOT generic here, and stays a subclass's problem if it ever
// needs to differ: every current and planned board on this class (AM243,
// Teensy, Arduino, ESP32) is a serial FrameTransport speaking STEP_DIR
// channels, so ValidateComm/CreateTransport below default to exactly that.
// Esp32Board overrides CreateTransport only to wait out the board's
// auto-reset after open; the handshake itself is unchanged.
//
// Board identity (expected BoardType / jw1_board_id_t) is constructor
// data, not a virtual hook: unlike ValidateComm/CreateTransport
// below (genuine behavior a future board might need to override), a
// board's identity is a compile-time-known constant with no logic behind
// it, so a subclass just passes it to this constructor — no vtable entry,
// no per-subclass .cc file needed for something that only ever returns a
// literal. See robot/board/teensy/teensy_board.h for how small that makes
// a concrete board.
class JoshuaWireBoard : public BoardInterface {
 public:
  JoshuaWireBoard(robot::board::BoardType expected_board_type,
                  jw1_board_id_t expected_wire_board_id)
      : expected_board_type_(expected_board_type),
        expected_wire_board_id_(expected_wire_board_id) {}
  ~JoshuaWireBoard() override = default;

  absl::Status Init(const robot::board::Board& config) final;
  absl::StatusOr<std::shared_ptr<BoardChannel>> OpenChannel(uint32_t index) final;
  absl::Status Teardown() final;

  // Replaces the FrameTransport so the IDENTIFY/CONFIGURE_CHANNEL handshake
  // and channel dispatch are testable without real hardware. Pass nullptr to restore each
  // subclass's real CreateTransport() path. Shared by every
  // subclass (one process-wide test seam, reset in TearDown) — safe as
  // long as tests run serially, which is gtest's default and already the
  // pattern this seam has always used.
  static void SetFrameTransportFactoryForTesting(
      std::function<absl::StatusOr<std::shared_ptr<FrameTransport>>(const robot::comm::Comm&)>
          factory);

 protected:
  // Checked before any transport is opened. Default requires SERIAL — the
  // only comm type any joshua_wire_v1 board uses today; override if a
  // future variant (e.g. a UDP/W5500 firmware build) needs a different
  // one.
  virtual absl::Status ValidateComm(const robot::comm::Comm& comm,
                                    const std::string& board_name) const;

  // Creates the configured message transport through CommFactory.
  virtual absl::StatusOr<std::shared_ptr<FrameTransport>> CreateTransport(
      const robot::comm::Comm& comm) const;

 private:
  // These two need expected_board_type_/expected_wire_board_id_, so
  // they're methods (not the free functions in the .cc's anonymous
  // namespace that everything else is) — everything they check beyond
  // that identity fact is generic across every joshua_wire_v1 board.
  absl::Status ValidateConfig(const robot::board::Board& config) const;
  absl::Status IdentifyAndValidate(FrameTransport& transport,
                                   const robot::board::Board& config) const;

  const robot::board::BoardType expected_board_type_;
  const jw1_board_id_t expected_wire_board_id_;

  bool initialized_ = false;
  robot::board::Board config_;
  std::shared_ptr<FrameTransport> transport_;
  std::map<uint32_t, std::shared_ptr<BoardChannel>> channels_;
};

}  // namespace robot::board
