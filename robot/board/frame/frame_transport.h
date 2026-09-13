#pragma once

#include "robot/comm/interfaces/message_transport.h"

namespace robot::board {

// Board-facing name for the generic atomic message capability.
using FrameTransport = robot::comm::MessageTransport;

}  // namespace robot::board
