load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "frame_transport",
    hdrs = ["frame_transport.h"],
    visibility = ["//visibility:public"],
    deps = ["//robot/comm/interfaces:message_transport"],
)

cc_library(
    name = "fake_frame_transport",
    testonly = True,
    hdrs = ["fake_frame_transport.h"],
    visibility = ["//visibility:public"],
    deps = [
        ":frame_transport",
        "@abseil-cpp//absl/status:status",
        "@abseil-cpp//absl/status:statusor",
    ],
)
