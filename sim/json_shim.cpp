// ============================================================================
// json_shim.cpp
// 本机 JSON shim：为 unitree::common::FromJsonString 提供实现。
//
// 为什么需要：
//   include/unitree/common/json/json.hpp 里只声明了 FromJsonString，
//   实现位于 Linux 预编译库 libunitree_sdk2.a（随 SDK 只提供 x86_64/aarch64
//   版本），macOS 本机无法链接。而 r1_pico_udp.h 的解析层依赖它。
//
// 做法：
//   FromJsonString 的返回类型 Any 与容器 JsonMap/JsonArray 全部是
//   header-only 的标准类型（std::map<std::string,Any> / std::vector<Any>），
//   所以只要补上这一个函数，r1_pico_udp.h 就能在本机完整编译运行。
//
//   DLS 的一致性约束：AnyNumberCast 依据源类型分派，故整数必须存成
//   int64/uint64、浮点存成 double，与官方实现保持同构（否则序列号等字段
//   的等值判断会退化）。
//
// 与官方实现的差异：
//   本 shim 只用于本机验证解析与对齐逻辑，不参与机器人上的构建。
//   官方库仍由 Linux 侧链接，二者互不影响。
// ============================================================================

#include <unitree/common/json/json.hpp>

#include <json.hpp>  // nlohmann/json 单头，位于 sim/thirdparty/

#include <string>

namespace unitree {
namespace common {

namespace {

using Json = nlohmann::json;

Any jsonToAny(const Json& j) {
  if (j.is_object()) {
    JsonMap m;
    for (auto it = j.begin(); it != j.end(); ++it) m[it.key()] = jsonToAny(it.value());
    return Any(m);
  }
  if (j.is_array()) {
    JsonArray a;
    a.reserve(j.size());
    for (const Json& v : j) a.push_back(jsonToAny(v));
    return Any(a);
  }
  if (j.is_string()) return Any(j.get<std::string>());
  if (j.is_boolean()) return Any(j.get<bool>());
  // 整数须与 AnyNumberCast 的类型分派匹配，不能退化成 double
  if (j.is_number_unsigned()) return Any(j.get<uint64_t>());
  if (j.is_number_integer()) return Any(j.get<int64_t>());
  if (j.is_number_float()) return Any(j.get<double>());
  return Any();  // null
}

}  // namespace

Any FromJsonString(const std::string& s) {
  // allow_exceptions=false：畸形 JSON 返回 discarded，由调用方按"解析失败"处理
  const Json j = Json::parse(s, nullptr, false);
  if (j.is_discarded()) return Any();
  return jsonToAny(j);
}

}  // namespace common
}  // namespace unitree
