#pragma once

#include <deque>
#include <vector>

#include "robot/board/frame/frame_transport.h"

namespace robot::board {

// In-memory message transport for board tests.
class FakeFrameTransport : public FrameTransport {
 public:
  absl::Status Open() override {
    return absl::OkStatus();
  }

  absl::Status Write(const std::vector<uint8_t>& message) override {
    send_calls_++;
    last_sent_ = message;
    sent_.push_back(message);
    return send_status_;
  }

  absl::StatusOr<std::vector<uint8_t>> SendAndReceive(const std::vector<uint8_t>& request_frame,
                                                      size_t expected_response_len) override {
    send_calls_++;
    last_sent_ = request_frame;
    sent_.push_back(request_frame);
    if (!send_status_.ok()) {
      return send_status_;
    }
    if (queued_responses_.empty()) {
      return std::vector<uint8_t>(expected_response_len, 0);
    }
    auto response = std::move(queued_responses_.front());
    queued_responses_.pop_front();
    return response;
  }

  void QueueResponse(std::vector<uint8_t> response) {
    queued_responses_.push_back(std::move(response));
  }

  int send_calls_ = 0;
  absl::Status send_status_ = absl::OkStatus();
  std::vector<uint8_t> last_sent_;
  std::vector<std::vector<uint8_t>> sent_;
  std::deque<std::vector<uint8_t>> queued_responses_;
};

}  // namespace robot::board
