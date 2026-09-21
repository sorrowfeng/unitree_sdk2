// macOS shim：Linux 专有头文件占位。
// unitree/common/decl.hpp 会 include 它但本机仿真链路并不使用其中符号，
// 提供空占位以便在 macOS 上编译解析层（仅用于 sim/ 验证，不参与机器人构建）。
#pragma once
