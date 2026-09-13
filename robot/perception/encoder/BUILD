load("@rules_cc//cc:defs.bzl", "cc_library")

cc_library(
    name = "sts3215_encoder",
    srcs = ["sts3215_encoder.cc"],
    hdrs = ["sts3215_encoder.h"],
    visibility = ["//visibility:public"],
    deps = [
        "//robot/comm/interfaces:message_transport",
        "//robot/perception/interfaces:encoder_interface",
        "//robot/perception/proto:perception_cc_proto",
        "//robot/perception/proto:perception_packet_cc_proto",
        "//utils:status_macros",
        "@glog",
    ],
)
