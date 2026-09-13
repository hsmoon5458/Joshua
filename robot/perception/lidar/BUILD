load("@rules_cc//cc:defs.bzl", "cc_library")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "lds01_driver",
    srcs = ["lds01_driver.cc"],
    hdrs = ["lds01_driver.h"],
    deps = [
        "//config/proto:robot_cc_proto",
        "//robot/comm/interfaces:byte_stream",
        "//robot/perception/interfaces:lidar_interface",
        "//robot/perception/proto:perception_packet_cc_proto",
        "//utils:status_macros",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@glog",
    ],
)
