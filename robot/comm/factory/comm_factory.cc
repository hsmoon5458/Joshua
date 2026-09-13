#include "robot/comm/factory/comm_factory.h"

#include <boost/asio.hpp>
#include <thread>

#include "robot/comm/ethercat/ethercat_transport.h"
#include "robot/comm/ethercat/soem_ethercat_transport.h"
#include "robot/comm/serial/serial.h"

namespace robot::comm {

namespace {
// Shared serial resources keyed by port and baud rate.
struct PortResources {
  std::shared_ptr<boost::asio::io_context> io_context{std::make_shared<boost::asio::io_context>()};
  boost::asio::executor_work_guard<boost::asio::io_context::executor_type> work_guard{
      boost::asio::make_work_guard(*io_context)};
  std::thread io_context_thread{[this] { io_context->run(); }};
  std::map<uint32_t, std::shared_ptr<robot::comm::Serial>> serials;
  ~PortResources() {
    work_guard.reset();
    if (io_context_thread.joinable()) io_context_thread.join();
  }
};

static std::mutex g_serial_mutex;
static std::map<std::string, std::unique_ptr<PortResources>> g_port_resources;  // keyed by port

// One SOEM master per interface, with a fixed process-data mode.
struct CachedEthercatTransport {
  robot::comm::ethercat::ProcessDataMode process_data_mode;
  std::shared_ptr<robot::comm::ethercat::EthercatTransport> transport;
};

static std::mutex g_ethercat_mutex;
static std::map<std::string, CachedEthercatTransport>
    g_ethercat_transports;  // keyed by interface name
static std::function<std::shared_ptr<robot::comm::ethercat::EthercatTransport>()>
    g_ethercat_transport_factory_for_testing;

absl::StatusOr<robot::comm::ethercat::ProcessDataMode> ToTransportProcessDataMode(
    EthercatProcessDataMode process_data_mode) {
  switch (process_data_mode) {
    case EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_SPLIT_LRD_LWR:
      return robot::comm::ethercat::ProcessDataMode::kSplitLrdLwr;
    case EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_LRW:
      return robot::comm::ethercat::ProcessDataMode::kLrw;
    case EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_INVALID:
    default:
      return absl::Status(absl::StatusCode::kInvalidArgument,
                          "EtherCAT config has invalid process data mode");
  }
}
}  // namespace

absl::StatusOr<CommTransport> CommFactory::CreateComm(const robot::comm::Comm& comm) {
  switch (comm.comm_type()) {
    case CommType::SERIAL: {
      if (!comm.has_serial_config()) {
        return absl::InvalidArgumentError("SERIAL comm has no serial_config.");
      }
      if (comm.transport_type() == TransportType::TRANSPORT_INVALID) {
        return absl::InvalidArgumentError("Comm has an invalid transport_type.");
      }
      if (comm.transport_type() == TransportType::CYCLIC) {
        return absl::InvalidArgumentError("SERIAL does not provide a cyclic transport.");
      }
      auto serial_or = CreateSerial(comm.serial_config());
      if (!serial_or.ok()) {
        return serial_or.status();
      }
      switch (comm.transport_type()) {
        case TransportType::BYTE_STREAM:
          return CommTransport{std::static_pointer_cast<ByteStream>(*serial_or)};
        case TransportType::MESSAGE:
          return CommTransport{std::static_pointer_cast<MessageTransport>(*serial_or)};
        case TransportType::CYCLIC:
        case TransportType::TRANSPORT_INVALID:
        default:
          return absl::InvalidArgumentError("Comm has an invalid transport_type.");
      }
    }
    case CommType::ETHERCAT: {
      if (comm.transport_type() != TransportType::CYCLIC) {
        return absl::InvalidArgumentError("ETHERCAT requires CYCLIC transport_type.");
      }
      if (!comm.has_ethercat_config()) {
        return absl::InvalidArgumentError("ETHERCAT comm has no ethercat_config.");
      }
      auto ethercat_or = CreateEthercat(comm.ethercat_config());
      if (!ethercat_or.ok()) {
        return ethercat_or.status();
      }
      return CommTransport{*ethercat_or};
    }
    case CommType::ETHERNET_UDP:
      if (comm.transport_type() != TransportType::MESSAGE) {
        return absl::InvalidArgumentError("ETHERNET_UDP requires MESSAGE transport_type.");
      }
      return absl::UnimplementedError("ETHERNET_UDP communication is not implemented.");
    case CommType::COMM_INVALID:
    default:
      return absl::InvalidArgumentError("Comm has an invalid comm_type.");
  }
}

absl::StatusOr<std::shared_ptr<Serial>> CommFactory::CreateSerial(
    const robot::comm::SerialConfig& config) {
  if (config.port().empty()) {
    return absl::Status(absl::StatusCode::kInvalidArgument, "Serial config has no port");
  }

  if (config.baudrate() == 0) {
    return absl::Status(absl::StatusCode::kInvalidArgument, "Serial config has no baudrate");
  }

  const std::string& port = config.port();
  uint32_t baudrate = config.baudrate();

  std::lock_guard<std::mutex> lock(g_serial_mutex);
  auto& port_res_ptr = g_port_resources[port];
  if (!port_res_ptr) {
    port_res_ptr = std::make_unique<PortResources>();
  }

  auto& serials = port_res_ptr->serials;
  auto it = serials.find(baudrate);
  if (it != serials.end()) {
    return it->second;
  }

  auto serial = std::make_shared<Serial>(port_res_ptr->io_context, port, baudrate);
  serials[baudrate] = serial;
  return serial;
}

absl::StatusOr<std::shared_ptr<robot::comm::ethercat::EthercatTransport>>
CommFactory::CreateEthercat(const robot::comm::EthercatConfig& config) {
  if (config.interface_name().empty()) {
    return absl::Status(absl::StatusCode::kInvalidArgument,
                        "EtherCAT config has no interface name");
  }

  auto process_data_mode_or = ToTransportProcessDataMode(config.process_data_mode());
  if (!process_data_mode_or.ok()) {
    return process_data_mode_or.status();
  }

  const std::string& interface_name = config.interface_name();

  std::lock_guard<std::mutex> lock(g_ethercat_mutex);
  auto it = g_ethercat_transports.find(interface_name);
  if (it != g_ethercat_transports.end()) {
    if (it->second.process_data_mode != *process_data_mode_or) {
      return absl::Status(absl::StatusCode::kInvalidArgument,
                          "EtherCAT interface " + interface_name +
                              " is already open with a different process data mode");
    }
    return it->second.transport;
  }

  std::shared_ptr<robot::comm::ethercat::EthercatTransport> transport;
  if (g_ethercat_transport_factory_for_testing) {
    transport = g_ethercat_transport_factory_for_testing();
  } else {
    transport = std::make_shared<robot::comm::ethercat::SoemEthercatTransport>();
  }
  auto status = transport->Init(interface_name, *process_data_mode_or);
  if (!status.ok()) {
    return status;
  }
  g_ethercat_transports[interface_name] = CachedEthercatTransport{*process_data_mode_or, transport};
  return transport;
}

void CommFactory::SetEthercatTransportFactoryForTesting(
    std::function<std::shared_ptr<robot::comm::ethercat::EthercatTransport>()> factory) {
  std::lock_guard<std::mutex> lock(g_ethercat_mutex);
  g_ethercat_transport_factory_for_testing = std::move(factory);
}

void CommFactory::ResetEthercatTransportCacheForTesting() {
  std::lock_guard<std::mutex> lock(g_ethercat_mutex);
  for (auto& [interface_name, cached] : g_ethercat_transports) {
    cached.transport->Teardown().IgnoreError();
  }
  g_ethercat_transports.clear();
}
}  // namespace robot::comm
