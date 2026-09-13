#pragma once

#include <glog/logging.h>

#include <boost/asio.hpp>
#include <boost/asio/serial_port_base.hpp>
#include <mutex>
#include <vector>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/comm/interfaces/byte_stream.h"
#include "robot/comm/interfaces/message_transport.h"

namespace robot::comm {

// Serial request/response capability with an atomic write-then-read operation
// for shared half-duplex buses.
class SerialTransport : public MessageTransport {
 public:
  virtual ~SerialTransport() = default;
  absl::Status Open() override {
    return absl::OkStatus();
  }
  virtual absl::Status Write(const std::vector<uint8_t>& data) override = 0;
  // Atomic Write-then-Read operation to prevent bus collisions.
  virtual absl::StatusOr<std::vector<uint8_t>> AtomicRead(const std::vector<uint8_t>& command,
                                                          size_t expected_response_size) = 0;

  absl::StatusOr<std::vector<uint8_t>> SendAndReceive(const std::vector<uint8_t>& request,
                                                      size_t expected_response_size) final {
    return AtomicRead(request, expected_response_size);
  }
};

// A serial mechanism can provide both an ordered byte stream and atomic
// request/response operations.
class Serial : public SerialTransport, public ByteStream {
 public:
  Serial(std::shared_ptr<boost::asio::io_context> io, std::string uart_port, int uart_baudrate);
  ~Serial();
  absl::Status Write(const std::vector<uint8_t>& data) override;
  absl::StatusOr<std::vector<uint8_t>> Read(size_t bytes_to_read) override;

  absl::StatusOr<std::vector<uint8_t>> AtomicRead(const std::vector<uint8_t>& command,
                                                  size_t expected_response_size) override;

  absl::Status Flush();
  absl::Status Open() override;

 private:
  std::string uart_port_;
  int uart_baudrate_;
  std::shared_ptr<boost::asio::io_context> io_context_;
  std::unique_ptr<boost::asio::serial_port> serial_;
  std::mutex mutex_;  // UART Bus can use single serial. To avoid race condition.
};
}  // namespace robot::comm
