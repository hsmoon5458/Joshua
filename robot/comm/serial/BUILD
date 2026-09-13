load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library")

cc_library(
    name = "serial",
    srcs = ["serial.cc"],
    hdrs = ["serial.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/comm/interfaces:byte_stream",
        "//robot/comm/interfaces:message_transport",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@boost.asio",
        "@glog",
    ],
)

cc_binary(
    name = "am243_demo_smoke",
    srcs = ["am243_demo_smoke.cc"],
    visibility = ["//visibility:public"],
    deps = [
        ":serial",
        "//firmware/common:joshua_wire_v1",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
        "@boost.asio",
    ],
)

cc_library(
    name = "fake_serial_transport",
    testonly = True,
    hdrs = ["fake_serial_transport.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":serial",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
    ],
)
