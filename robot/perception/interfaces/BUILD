load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "perception_interface",
    hdrs = ["perception_interface.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/perception/proto:perception_packet_cc_proto",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
    ],
)

cc_library(
    name = "camera_interface",
    hdrs = ["camera_interface.h"],
    deps = [":perception_interface"],
)

cc_library(
    name = "lidar_interface",
    hdrs = ["lidar_interface.h"],
    deps = [":perception_interface"],
)

cc_library(
    name = "imu_interface",
    hdrs = ["imu_interface.h"],
    deps = [":perception_interface"],
)

cc_library(
    name = "position_interface",
    hdrs = ["position_interface.h"],
    deps = [":perception_interface"],
)

cc_library(
    name = "radar_interface",
    hdrs = ["radar_interface.h"],
    deps = [":perception_interface"],
)
