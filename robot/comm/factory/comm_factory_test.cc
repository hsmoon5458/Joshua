#include "robot/comm/factory/comm_factory.h"

#include <memory>

#include "absl/status/status.h"
#include "gtest/gtest.h"
#include "robot/comm/ethercat/fake_ethercat_transport.h"
#include "robot/comm/proto/comm.pb.h"

namespace robot::comm {
namespace {

using robot::comm::ethercat::FakeEthercatTransport;

robot::comm::Comm MakeEthercatComm() {
  robot::comm::Comm comm;
  comm.set_comm_type(robot::comm::CommType::ETHERCAT);
  comm.set_transport_type(robot::comm::TransportType::CYCLIC);
  auto* config = comm.mutable_ethercat_config();
  config->set_interface_name("joshua-no-such-ethercat-iface0");
  config->set_process_data_mode(
      robot::comm::EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_SPLIT_LRD_LWR);
  return comm;
}

TEST(CommFactoryTest, CreateCommRejectsMissingTransportType) {
  robot::comm::Comm comm;
  comm.set_comm_type(robot::comm::CommType::SERIAL);
  comm.mutable_serial_config()->set_port("/dev/ttyUSB0");
  comm.mutable_serial_config()->set_baudrate(115200);

  auto transport = CommFactory::CreateComm(comm);

  EXPECT_EQ(transport.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(CommFactoryTest, CreateCommRejectsUnsupportedMechanismCapabilityPair) {
  robot::comm::Comm comm;
  comm.set_comm_type(robot::comm::CommType::ETHERNET_UDP);
  comm.set_transport_type(robot::comm::TransportType::BYTE_STREAM);

  auto transport = CommFactory::CreateComm(comm);

  EXPECT_EQ(transport.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(CommFactoryTest, CreateCommRejectsMissingEthercatConfig) {
  robot::comm::Comm comm;
  comm.set_comm_type(robot::comm::CommType::ETHERCAT);
  comm.set_transport_type(robot::comm::TransportType::CYCLIC);

  auto transport_or = CommFactory::CreateComm(comm);

  EXPECT_EQ(transport_or.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(CommFactoryTest, CreateEthercatRejectsMissingInterfaceName) {
  auto comm = MakeEthercatComm();
  comm.mutable_ethercat_config()->clear_interface_name();

  auto transport_or = CommFactory::CreateEthercat(comm.ethercat_config());

  EXPECT_EQ(transport_or.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(CommFactoryTest, CreateEthercatRejectsInvalidProcessDataMode) {
  auto comm = MakeEthercatComm();
  comm.mutable_ethercat_config()->set_process_data_mode(
      robot::comm::EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_INVALID);

  auto transport_or = CommFactory::CreateEthercat(comm.ethercat_config());

  EXPECT_EQ(transport_or.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST(CommFactoryTest, CreateEthercatReportsUnavailableForMissingInterface) {
  auto comm = MakeEthercatComm();
  auto transport_or = CommFactory::CreateEthercat(comm.ethercat_config());

  EXPECT_EQ(transport_or.status().code(), absl::StatusCode::kUnavailable);
}

class CommFactoryEthercatCacheTest : public ::testing::Test {
 protected:
  void SetUp() override {
    CommFactory::SetEthercatTransportFactoryForTesting(
        [] { return std::make_shared<FakeEthercatTransport>(); });
  }

  void TearDown() override {
    CommFactory::SetEthercatTransportFactoryForTesting(nullptr);
    CommFactory::ResetEthercatTransportCacheForTesting();
  }
};

TEST_F(CommFactoryEthercatCacheTest, SameInterfaceSharesOneMaster) {
  auto comm = MakeEthercatComm();
  auto first_or = CommFactory::CreateEthercat(comm.ethercat_config());
  auto second_or = CommFactory::CreateEthercat(comm.ethercat_config());

  ASSERT_TRUE(first_or.ok()) << first_or.status();
  ASSERT_TRUE(second_or.ok()) << second_or.status();
  EXPECT_EQ(first_or->get(), second_or->get());
}

TEST_F(CommFactoryEthercatCacheTest, DifferentInterfacesGetDifferentMasters) {
  auto first_comm = MakeEthercatComm();
  auto first_or = CommFactory::CreateEthercat(first_comm.ethercat_config());
  auto other_comm = MakeEthercatComm();
  other_comm.mutable_ethercat_config()->set_interface_name("joshua-no-such-ethercat-iface1");
  auto second_or = CommFactory::CreateEthercat(other_comm.ethercat_config());

  ASSERT_TRUE(first_or.ok()) << first_or.status();
  ASSERT_TRUE(second_or.ok()) << second_or.status();
  EXPECT_NE(first_or->get(), second_or->get());
}

TEST_F(CommFactoryEthercatCacheTest, RejectsProcessDataModeChangeOnOpenInterface) {
  auto first_comm = MakeEthercatComm();
  auto first_or = CommFactory::CreateEthercat(first_comm.ethercat_config());
  ASSERT_TRUE(first_or.ok()) << first_or.status();

  auto lrw_comm = MakeEthercatComm();
  lrw_comm.mutable_ethercat_config()->set_process_data_mode(
      robot::comm::EthercatProcessDataMode::ETHERCAT_PROCESS_DATA_MODE_LRW);
  auto second_or = CommFactory::CreateEthercat(lrw_comm.ethercat_config());

  EXPECT_EQ(second_or.status().code(), absl::StatusCode::kInvalidArgument);
}

TEST_F(CommFactoryEthercatCacheTest, FailedInitIsNotCached) {
  int factory_calls = 0;
  CommFactory::SetEthercatTransportFactoryForTesting([&factory_calls] {
    factory_calls++;
    auto transport = std::make_shared<FakeEthercatTransport>();
    if (factory_calls == 1) {
      transport->init_status_ = absl::Status(absl::StatusCode::kUnavailable, "no NIC");
    }
    return transport;
  });

  auto comm = MakeEthercatComm();
  auto failed_or = CommFactory::CreateEthercat(comm.ethercat_config());
  EXPECT_EQ(failed_or.status().code(), absl::StatusCode::kUnavailable);

  auto retry_or = CommFactory::CreateEthercat(comm.ethercat_config());
  EXPECT_TRUE(retry_or.ok()) << retry_or.status();
  EXPECT_EQ(factory_calls, 2);
}

}  // namespace
}  // namespace robot::comm
