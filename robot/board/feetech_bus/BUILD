load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "feetech_protocol",
    srcs = ["feetech_protocol.cc"],
    hdrs = ["feetech_protocol.h"],
    visibility = ["//visibility:public"],
    deps = [
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "feetech_bus_board",
    srcs = ["feetech_bus_board.cc"],
    hdrs = ["feetech_bus_board.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":feetech_protocol",
        "//robot/board/interfaces:board_interfaces",
        "//robot/board/proto:board_cc_proto",
        "//robot/comm/factory:comm_factory",
        "//robot/comm/interfaces:message_transport",
        "//robot/comm/proto:comm_cc_proto",
        "//utils:status_macros",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_test(
    name = "feetech_protocol_test",
    srcs = ["feetech_protocol_test.cc"],
    deps = [
        ":feetech_protocol",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "feetech_bus_board_test",
    srcs = ["feetech_bus_board_test.cc"],
    deps = [
        ":feetech_bus_board",
        ":feetech_protocol",
        "//robot/board/proto:board_cc_proto",
        "//robot/comm/proto:comm_cc_proto",
        "//robot/comm/serial:fake_serial_transport",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)
