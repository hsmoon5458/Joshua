#pragma once

#include <functional>
#include <map>
#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/board/interfaces/board_interface.h"
#include "robot/board/proto/board.pb.h"
#include "robot/comm/interfaces/message_transport.h"
#include "robot/comm/proto/comm.pb.h"

namespace robot::board {

// Transport handle and the bus mutex, shared between the board and every
// open channel so a channel outliving the board stays safe. Defined in
// feetech_bus_board.cc.
struct FeetechBusSharedState;

// BoardType::FEETECH_BUS — a degenerate board where the comm leg (a serial
// port) and the drive leg (the servo's own MCU) are the same physical link;
// each daisy-chained servo is a channel keyed by servo_id
// Init pings every configured servo and reads its model-number register so wiring/config mismatches
// surface at startup instead of on the first move. The register protocol
// itself lives in feetech_protocol.h; this class owns bus serialization (one
// request/response in flight on the half-duplex UART) via the shared mutex.
class FeetechBusBoard : public BoardInterface {
 public:
  FeetechBusBoard() = default;

  absl::Status Init(const robot::board::Board& config) override;
  absl::StatusOr<std::shared_ptr<BoardChannel>> OpenChannel(uint32_t index) override;
  absl::Status Teardown() override;

  // Replaces the Serial connection so bus behavior is testable without a
  // real port. Pass nullptr to restore the default CommFactory path. For tests.
  static void SetSerialTransportFactoryForTesting(
      std::function<absl::StatusOr<std::shared_ptr<robot::comm::MessageTransport>>(
          const robot::comm::Comm&)> factory);

 private:
  bool initialized_ = false;
  robot::board::Board config_;
  std::shared_ptr<FeetechBusSharedState> state_;
  std::map<uint32_t, std::shared_ptr<BoardChannel>> channels_;
};

}  // namespace robot::board
