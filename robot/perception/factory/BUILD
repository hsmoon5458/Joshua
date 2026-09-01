load("@rules_cc//cc:defs.bzl", "cc_library", "cc_test")

cc_library(
    name = "perception_factory",
    hdrs = ["perception_factory.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//config/proto:robot_cc_proto",
        "//robot/board/factory:board_factory",
        "//robot/board/factory:board_resolver",
        "//robot/board/factory:sensor_channel_validation",
        "//robot/comm/factory:comm_factory",
        "//robot/perception/camera:cv_camera",
        "//robot/perception/encoder:sts3215_encoder",
        "//robot/perception/interfaces:perception_interface",
        "//robot/perception/lidar:lds01_driver",
        "//robot/perception/sensors:joint_position_sensor",
        "//utils:status_macros",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@abseil-cpp//absl/strings",
    ],
)

cc_test(
    name = "perception_factory_cc_test",
    srcs = ["perception_factory_test.cc"],
    deps = [
        ":perception_factory",
        "//config/proto:robot_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@googletest//:gtest_main",
    ],
)
