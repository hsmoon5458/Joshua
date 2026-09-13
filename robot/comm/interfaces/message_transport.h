#pragma once

#include <cstdint>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace robot::comm {

// Atomic message-oriented communication. Implementations preserve a complete
// request while waiting for its response so shared links cannot interleave.
class MessageTransport {
 public:
  virtual ~MessageTransport() = default;

  virtual absl::Status Open() = 0;
  virtual absl::Status Write(const std::vector<uint8_t>& message) = 0;
  virtual absl::StatusOr<std::vector<uint8_t>> SendAndReceive(const std::vector<uint8_t>& request,
                                                              size_t expected_response_size) = 0;
};

}  // namespace robot::comm
