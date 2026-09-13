#pragma once

#include <memory>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/board/interfaces/board_channel.h"
#include "robot/board/proto/board.pb.h"

namespace robot::board {

// Shared interface to an addressable hardware controller.
class BoardInterface {
 public:
  virtual ~BoardInterface() = default;
  // Opens comm, runs the IDENTIFY handshake, pushes CONFIGURE_CHANNEL setup.
  virtual absl::Status Init(const robot::board::Board& config) = 0;
  virtual absl::StatusOr<std::shared_ptr<BoardChannel>> OpenChannel(uint32_t index) = 0;
  virtual absl::Status Teardown() = 0;
};

}  // namespace robot::board
