load("@rules_cc//cc:defs.bzl", "cc_library")
load("@rules_cc//cc:defs.bzl", "cc_test")

cc_library(
    name = "board_factory",
    srcs = ["board_factory.cc"],
    hdrs = ["board_factory.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/board/am243:am243_board",
        "//robot/board/esp32:esp32_board",
        "//robot/board/feetech_bus:feetech_bus_board",
        "//robot/board/interfaces:board_interfaces",
        "//robot/board/mock:mock_board",
        "//robot/board/proto:board_cc_proto",
        "//robot/board/teensy:teensy_board",
        "//utils:status_macros",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "motor_channel_validation",
    srcs = ["motor_channel_validation.cc"],
    hdrs = ["motor_channel_validation.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/action/proto:action_cc_proto",
        "//robot/board/proto:board_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "board_resolver",
    srcs = ["board_resolver.cc"],
    hdrs = ["board_resolver.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/board/proto:board_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_library(
    name = "sensor_channel_validation",
    srcs = ["sensor_channel_validation.cc"],
    hdrs = ["sensor_channel_validation.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/board/proto:board_cc_proto",
        "//robot/perception/proto:perception_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/strings",
    ],
)

cc_test(
    name = "sensor_channel_validation_test",
    srcs = ["sensor_channel_validation_test.cc"],
    deps = [
        ":sensor_channel_validation",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "board_factory_test",
    srcs = ["board_factory_test.cc"],
    deps = [
        ":board_factory",
        "//robot/board/proto:board_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)

cc_test(
    name = "motor_channel_validation_test",
    srcs = ["motor_channel_validation_test.cc"],
    deps = [
        ":motor_channel_validation",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)
