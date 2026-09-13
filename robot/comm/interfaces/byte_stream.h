#pragma once

#include <cstddef>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"

namespace robot::comm {

// An ordered sequence of bytes with no inherent message boundaries. This is
// a capability, not a configured comm type: serial and TCP naturally provide
// it, while datagram and cyclic links do not. Framed message protocols may be
// layered over the same byte stream without changing this low-level contract.
class ByteStream {
 public:
  virtual ~ByteStream() = default;
  virtual absl::Status Open() = 0;
  virtual absl::Status Write(const std::vector<uint8_t>& data) = 0;
  virtual absl::StatusOr<std::vector<uint8_t>> Read(size_t bytes_to_read) = 0;
};

}  // namespace robot::comm
