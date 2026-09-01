#pragma once

#include "absl/status/status.h"
#include "robot/board/proto/board.pb.h"
#include "robot/perception/proto/perception.pb.h"

namespace robot::board {

// Validates that a signal interface can produce the requested sensor value.
absl::Status ValidateSensorChannel(robot::perception::SensorType sensor_type,
                                   robot::board::SignalInterface signal);

}  // namespace robot::board
