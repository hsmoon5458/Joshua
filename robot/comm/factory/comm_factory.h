#pragma once

#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <utility>
#include <variant>

#include "absl/status/status.h"
#include "absl/status/statusor.h"
#include "robot/comm/ethercat/ethercat_transport.h"
#include "robot/comm/interfaces/byte_stream.h"
#include "robot/comm/interfaces/message_transport.h"
#include "robot/comm/proto/comm.pb.h"
#include "robot/comm/serial/serial.h"

namespace robot::comm {

using CommTransport = std::variant<std::shared_ptr<ByteStream>,
                                   std::shared_ptr<MessageTransport>,
                                   std::shared_ptr<robot::comm::ethercat::EthercatTransport>>;

template <typename Transport>
absl::StatusOr<std::shared_ptr<Transport>> GetCommTransport(const CommTransport& transport) {
  const auto* selected = std::get_if<std::shared_ptr<Transport>>(&transport);
  if (selected == nullptr) {
    return absl::InvalidArgumentError("Configured comm does not provide the requested transport.");
  }
  return *selected;
}

class CommFactory {
 public:
  static absl::StatusOr<CommTransport> CreateComm(const robot::comm::Comm& config);

  static absl::StatusOr<std::shared_ptr<Serial>> CreateSerial(
      const robot::comm::SerialConfig& config);

  // Returns a cached instance per interface name — an EtherCAT NIC has
  // exactly one master, and two ecx_init()s on one NIC fight over the raw
  // socket. Later calls return the same transport and fail if they request a
  // different process-data mode.
  static absl::StatusOr<std::shared_ptr<robot::comm::ethercat::EthercatTransport>> CreateEthercat(
      const robot::comm::EthercatConfig& config);

  // Replaces the SOEM transport constructor so cache semantics are testable
  // without a NIC. Pass nullptr to restore the default. For tests.
  static void SetEthercatTransportFactoryForTesting(
      std::function<std::shared_ptr<robot::comm::ethercat::EthercatTransport>()> factory);

  // Tears down and forgets every cached EtherCAT transport. For tests.
  static void ResetEthercatTransportCacheForTesting();

  ~CommFactory() = default;
  CommFactory(const CommFactory&) = delete;
  CommFactory& operator=(const CommFactory&) = delete;
  CommFactory(CommFactory&&) = default;
  CommFactory& operator=(CommFactory&&) = default;

 private:
  CommFactory() = default;
};
}  // namespace robot::comm
