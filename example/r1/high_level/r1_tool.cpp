// ============================================================================
// r1_tool.cpp
// R1 官方 SDK 全部服务命令行测试工具（二次开发前验证用）。
//
// 覆盖本 SDK 为 R1 提供的所有高层服务：
//   loco   -> r1::LocoClient            (DDS 服务 "sport")
//   msc    -> b2::MotionSwitcherClient   ("motion_switcher")
//   state  -> b2::RobotStateClient       ("robot_state")
//   audio  -> r1::AudioClient            ("voice")
//   config -> b2::ConfigClient           ("config")
//   lowstate -> 订阅 "rt/lowstate"（只读，安全）
//
// 用法：
//   r1_tool <network_interface> <group> <action> [args...]
//   r1_tool help
//
// 例：
//   r1_tool eth0 loco get-fsm
//   r1_tool eth0 loco stand
//   r1_tool eth0 msc check
//   r1_tool eth0 state services
//   r1_tool eth0 audio tts "hello"
//   r1_tool eth0 lowstate dump 3
// ============================================================================

#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "unitree/robot/channel/channel_factory.hpp"
#include "unitree/robot/channel/channel_subscriber.hpp"
#include "unitree/idl/hg/LowState_.hpp"
#include "unitree/idl/ros2/String_.hpp"

#include "unitree/robot/r1/loco/r1_loco_client.hpp"
#include "unitree/robot/r1/audio/audio_client.hpp"
#include "unitree/robot/b2/motion_switcher/motion_switcher_client.hpp"
#include "unitree/robot/b2/robot_state/robot_state_client.hpp"
#include "unitree/robot/b2/config/config_client.hpp"

using std::string;
using std::vector;

static void print_usage() {
  printf(
    "R1 官方 SDK 全部服务测试工具\n"
    "\n"
    "用法: r1_tool <network_interface> <group> <action> [args...]\n"
    "      r1_tool help\n"
    "\n"
    "group/action:\n"
    "  loco get-fsm                         查询 FSM ID        (sport 7001)\n"
    "  loco get-fsm-mode                    查询 FSM Mode      (sport 7002)\n"
    "  loco set-fsm <id>                    设置 FSM ID        (sport 7101)\n"
    "  loco damp                            阻尼  fsm 1\n"
    "  loco stand                           站立  fsm 4\n"
    "  loco start                           启动主运控 fsm 811\n"
    "  loco zero-torque                     零力矩 fsm 0 (危险)\n"
    "  loco stop                            停止移动\n"
    "  loco move <vx> <vy> <vyaw> [dur]     速度移动           (sport 7105)\n"
    "  loco speed <mode>                    速度档位           (sport 7107)\n"
    "\n"
    "  msc check                            查询当前运动模式   (motion_switcher 1001)\n"
    "  msc select <name|alias>              选择模式（如 ai）  (1002)\n"
    "  msc release                          释放当前模式       (1003)\n"
    "  msc silent-get                       查询静默状态       (1005)\n"
    "  msc silent-set <0|1>                 设置静默           (1004)\n"
    "\n"
    "  state services                       列出所有服务及状态 (robot_state 1003)\n"
    "  state version                        查询版本           (1006)\n"
    "  state lowpower-get                   低功耗状态         (1005)\n"
    "  state lowpower-set <0|1>             低功耗开关         (1004)\n"
    "  state report <interval_ms> <dur_ms>  设置上报频率       (1002)\n"
    "  state switch <name> <0|1>            服务开关           (1001)\n"
    "\n"
    "  audio tts <text> [speaker]           语音合成           (voice 1001)\n"
    "  audio vol-get                        查询音量           (1005)\n"
    "  audio vol-set <0-100>                设置音量           (1006)\n"
    "  audio led <r> <g> <b>                RGB 灯             (1010)\n"
    "  audio asr [sec]                      监听 ASR 识别结果  (topic rt/audio_msg)\n"
    "\n"
    "  config get <name>                    读取配置           (config 1002)\n"
    "  config set <name> <content>          写配置             (1001)\n"
    "  config del <name>                    删除配置           (1003)\n"
    "  config meta <name>                   配置元信息         (1004)\n"
    "\n"
    "  lowstate dump [sec]                  订阅 rt/lowstate 打印 IMU + 35 电机状态\n"
    "\n"
    "注意：危险动作（zero-torque / move）请确保机器人周围安全、有保护措施。\n");
}

static int need(const vector<string>& a, size_t n, const char* usage) {
  if (a.size() < n) { fprintf(stderr, "参数不足: %s\n", usage); return -1; }
  return 0;
}

// ---------------------------------------------------------------------------
// loco
// ---------------------------------------------------------------------------
static int cmd_loco(const vector<string>& a) {
  if (a.empty()) { fprintf(stderr, "用法: loco <action>\n"); return 1; }
  unitree::robot::r1::LocoClient c;
  c.Init();
  c.SetTimeout(5.0f);
  const string& act = a[0];
  int r = 0, id = 0, mode = 0;

  if (act == "get-fsm") {
    r = c.GetFsmId(id);
    printf("GetFsmId ret=%d fsm_id=%d\n", r, id);
  } else if (act == "get-fsm-mode") {
    r = c.GetFsmMode(mode);
    printf("GetFsmMode ret=%d fsm_mode=%d\n", r, mode);
  } else if (act == "set-fsm") {
    if (need(a, 2, "loco set-fsm <id>")) return 1;
    id = std::atoi(a[1].c_str());
    r = c.SetFsmId(id);
    printf("SetFsmId(%d) ret=%d\n", id, r);
  } else if (act == "damp") {
    r = c.Damp();            printf("Damp ret=%d\n", r);
  } else if (act == "stand") {
    r = c.StandUp();         printf("StandUp ret=%d\n", r);
  } else if (act == "start") {
    r = c.Start();           printf("Start ret=%d\n", r);
  } else if (act == "zero-torque") {
    r = c.ZeroTorque();      printf("ZeroTorque ret=%d\n", r);
  } else if (act == "stop") {
    r = c.StopMove();        printf("StopMove ret=%d\n", r);
  } else if (act == "move") {
    if (need(a, 4, "loco move <vx> <vy> <vyaw> [dur]")) return 1;
    const float vx = std::atof(a[1].c_str());
    const float vy = std::atof(a[2].c_str());
    const float vyaw = std::atof(a[3].c_str());
    const float dur = (a.size() > 4) ? std::atof(a[4].c_str()) : 1.0f;
    r = c.SetVelocity(vx, vy, vyaw, dur);
    printf("SetVelocity(%.3f %.3f %.3f, %.3fs) ret=%d\n", vx, vy, vyaw, dur, r);
  } else if (act == "speed") {
    if (need(a, 2, "loco speed <mode>")) return 1;
    mode = std::atoi(a[1].c_str());
    r = c.SetSpeedMode(mode);
    printf("SetSpeedMode(%d) ret=%d\n", mode, r);
  } else {
    fprintf(stderr, "未知 loco action: %s\n", act.c_str());
    return 1;
  }
  return r == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------
// msc (motion_switcher)
// ---------------------------------------------------------------------------
static int cmd_msc(const vector<string>& a) {
  if (a.empty()) { fprintf(stderr, "用法: msc <action>\n"); return 1; }
  unitree::robot::b2::MotionSwitcherClient c;
  c.Init();
  c.SetTimeout(5.0f);
  const string& act = a[0];
  int r = 0;

  if (act == "check") {
    string form, name;
    r = c.CheckMode(form, name);
    printf("CheckMode ret=%d form=\"%s\" name=\"%s\"\n", r, form.c_str(), name.c_str());
    if (name.empty()) printf("(当前无激活的运动模式；ai_sport 若在运行此处可能显示为 ai)\n");
  } else if (act == "select") {
    if (need(a, 2, "msc select <name|alias>")) return 1;
    r = c.SelectMode(a[1]);
    printf("SelectMode(%s) ret=%d\n", a[1].c_str(), r);
  } else if (act == "release") {
    r = c.ReleaseMode();
    printf("ReleaseMode ret=%d\n", r);
  } else if (act == "silent-get") {
    bool s = false;
    r = c.GetSilent(s);
    printf("GetSilent ret=%d silent=%d\n", r, s ? 1 : 0);
  } else if (act == "silent-set") {
    if (need(a, 2, "msc silent-set <0|1>")) return 1;
    r = c.SetSilent(std::atoi(a[1].c_str()) != 0);
    printf("SetSilent(%s) ret=%d\n", a[1].c_str(), r);
  } else {
    fprintf(stderr, "未知 msc action: %s\n", act.c_str());
    return 1;
  }
  return r == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------
// state (robot_state)
// ---------------------------------------------------------------------------
static int cmd_state(const vector<string>& a) {
  if (a.empty()) { fprintf(stderr, "用法: state <action>\n"); return 1; }
  unitree::robot::b2::RobotStateClient c;
  c.Init();
  c.SetTimeout(5.0f);
  const string& act = a[0];
  int r = 0;

  if (act == "services") {
    vector<unitree::robot::b2::ServiceState> list;
    r = c.ServiceList(list);
    printf("ServiceList ret=%d count=%zu\n", r, list.size());
    for (auto& s : list)
      printf("  %-24s status=%d protect=%d\n", s.name.c_str(), s.status, s.protect);
  } else if (act == "version") {
    string pkg;
    std::map<string, string> mods;
    r = c.GetPkgVersion(pkg, mods);
    printf("GetPkgVersion ret=%d packageVersion=%s\n", r, pkg.c_str());
    for (auto& kv : mods) printf("  %-24s %s\n", kv.first.c_str(), kv.second.c_str());
  } else if (act == "lowpower-get") {
    int32_t st = 0;
    r = c.LowPowerStatus(st);
    printf("LowPowerStatus ret=%d status=%d\n", r, st);
  } else if (act == "lowpower-set") {
    if (need(a, 2, "state lowpower-set <0|1>")) return 1;
    r = c.LowPowerSwitch(std::atoi(a[1].c_str()));
    printf("LowPowerSwitch(%s) ret=%d\n", a[1].c_str(), r);
  } else if (act == "report") {
    if (need(a, 3, "state report <interval_ms> <dur_ms>")) return 1;
    r = c.SetReportFreq(std::atoi(a[1].c_str()), std::atoi(a[2].c_str()));
    printf("SetReportFreq(%s, %s) ret=%d\n", a[1].c_str(), a[2].c_str(), r);
  } else if (act == "switch") {
    if (need(a, 3, "state switch <name> <0|1>")) return 1;
    int32_t status = 0;
    r = c.ServiceSwitch(a[1], std::atoi(a[2].c_str()), status);
    printf("ServiceSwitch(%s, %s) ret=%d status=%d\n", a[1].c_str(), a[2].c_str(), r, status);
  } else {
    fprintf(stderr, "未知 state action: %s\n", act.c_str());
    return 1;
  }
  return r == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------
// audio (voice)
// ---------------------------------------------------------------------------
static int cmd_audio(const vector<string>& a) {
  if (a.empty()) { fprintf(stderr, "用法: audio <action>\n"); return 1; }
  unitree::robot::r1::AudioClient c;
  c.Init();
  c.SetTimeout(5.0f);
  const string& act = a[0];
  int r = 0;

  if (act == "tts") {
    if (need(a, 2, "audio tts <text> [speaker]")) return 1;
    const int speaker = (a.size() > 2) ? std::atoi(a[2].c_str()) : 0;
    r = c.TtsMaker(a[1], speaker);
    printf("TtsMaker ret=%d speaker=%d text=\"%s\"\n", r, speaker, a[1].c_str());
  } else if (act == "vol-get") {
    uint8_t v = 0;
    r = c.GetVolume(v);
    printf("GetVolume ret=%d volume=%u\n", r, v);
  } else if (act == "vol-set") {
    if (need(a, 2, "audio vol-set <0-100>")) return 1;
    r = c.SetVolume(static_cast<uint8_t>(std::atoi(a[1].c_str())));
    printf("SetVolume(%s) ret=%d\n", a[1].c_str(), r);
  } else if (act == "led") {
    if (need(a, 4, "audio led <r> <g> <b>")) return 1;
    r = c.LedControl(static_cast<uint8_t>(std::atoi(a[1].c_str())),
                     static_cast<uint8_t>(std::atoi(a[2].c_str())),
                     static_cast<uint8_t>(std::atoi(a[3].c_str())));
    printf("LedControl(%s,%s,%s) ret=%d\n", a[1].c_str(), a[2].c_str(), a[3].c_str(), r);
  } else if (act == "asr") {
    const double secs = (a.size() > 1) ? std::atof(a[1].c_str()) : 10.0;
    printf("监听 ASR (rt/audio_msg) %g 秒...\n", secs);
    unitree::robot::ChannelSubscriber<std_msgs::msg::dds_::String_> sub("rt/audio_msg");
    sub.InitChannel([](const void* msg) {
      auto* m = static_cast<const std_msgs::msg::dds_::String_*>(msg);
      printf("[asr] %s\n", m->data().c_str());
    });
    std::this_thread::sleep_for(std::chrono::duration<double>(secs));
    sub.CloseChannel();
  } else {
    fprintf(stderr, "未知 audio action: %s\n", act.c_str());
    return 1;
  }
  return r == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------
// config
// ---------------------------------------------------------------------------
static int cmd_config(const vector<string>& a) {
  if (a.empty()) { fprintf(stderr, "用法: config <action>\n"); return 1; }
  unitree::robot::b2::ConfigClient c;
  c.Init();
  c.SetTimeout(5.0f);
  const string& act = a[0];
  int r = 0;

  if (act == "get") {
    if (need(a, 2, "config get <name>")) return 1;
    string content;
    r = c.Get(a[1], content);
    printf("Get(%s) ret=%d content=%s\n", a[1].c_str(), r, content.c_str());
  } else if (act == "set") {
    if (need(a, 3, "config set <name> <content>")) return 1;
    r = c.Set(a[1], a[2]);
    printf("Set(%s) ret=%d\n", a[1].c_str(), r);
  } else if (act == "del") {
    if (need(a, 2, "config del <name>")) return 1;
    r = c.Del(a[1]);
    printf("Del(%s) ret=%d\n", a[1].c_str(), r);
  } else if (act == "meta") {
    if (need(a, 2, "config meta <name>")) return 1;
    string meta;
    r = c.Meta(a[1], meta);
    printf("Meta(%s) ret=%d meta=%s\n", a[1].c_str(), r, meta.c_str());
  } else {
    fprintf(stderr, "未知 config action: %s\n", act.c_str());
    return 1;
  }
  return r == 0 ? 0 : 2;
}

// ---------------------------------------------------------------------------
// lowstate（只读）
// ---------------------------------------------------------------------------
namespace {
std::mutex g_low_mtx;
unitree_hg::msg::dds_::LowState_ g_low;
std::atomic<bool> g_low_have{false};

void lowstate_handler(const void* msg) {
  std::lock_guard<std::mutex> lk(g_low_mtx);
  g_low = *static_cast<const unitree_hg::msg::dds_::LowState_*>(msg);
  g_low_have = true;
}
}  // namespace

static int cmd_lowstate(const vector<string>& a) {
  if (a.empty() || a[0] != "dump") {
    fprintf(stderr, "用法: lowstate dump [sec]\n");
    return 1;
  }
  const double secs = (a.size() > 1) ? std::atof(a[1].c_str()) : 3.0;

  unitree::robot::ChannelSubscriber<unitree_hg::msg::dds_::LowState_> sub("rt/lowstate");
  sub.InitChannel(lowstate_handler);

  printf("等待 rt/lowstate ...\n");
  auto t0 = std::chrono::steady_clock::now();
  while (!g_low_have.load() && std::chrono::steady_clock::now() - t0 < std::chrono::seconds(5)) {
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
  }
  if (!g_low_have.load()) {
    printf("未收到 rt/lowstate（机器人未连接/网卡不对）\n");
    sub.CloseChannel();
    return 2;
  }

  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(secs);
  while (std::chrono::steady_clock::now() < deadline) {
    unitree_hg::msg::dds_::LowState_ s;
    { std::lock_guard<std::mutex> lk(g_low_mtx); s = g_low; }
    const auto& imu = s.imu_state();
    printf("---- mode_machine=%u  rpy=[%.3f %.3f %.3f]  gyro=[%.3f %.3f %.3f]  acc=[%.3f %.3f %.3f]\n",
           static_cast<unsigned>(s.mode_machine()),
           imu.rpy()[0], imu.rpy()[1], imu.rpy()[2],
           imu.gyroscope()[0], imu.gyroscope()[1], imu.gyroscope()[2],
           imu.accelerometer()[0], imu.accelerometer()[1], imu.accelerometer()[2]);
    printf("     idx:  q       dq      tau_est   mode\n");
    for (int i = 0; i < 35; ++i) {
      const auto& m = s.motor_state()[i];
      printf("     %2d:  %7.3f %7.3f %7.3f   %u\n", i, m.q(), m.dq(), m.tau_est(),
             static_cast<unsigned>(m.mode()));
    }
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
  sub.CloseChannel();
  return 0;
}

// ---------------------------------------------------------------------------
int main(int argc, char** argv) {
  if (argc < 2) { print_usage(); return 1; }
  const string first = argv[1];
  if (first == "help" || first == "-h" || first == "--help") { print_usage(); return 0; }

  if (argc < 3) {
    fprintf(stderr, "用法: r1_tool <network_interface> <group> <action> [args...]\n");
    return 1;
  }
  const string iface = argv[1];
  const string group = argv[2];
  vector<string> args(argv + 3, argv + argc);

  unitree::robot::ChannelFactory::Instance()->Init(0, iface);

  if (group == "loco")      return cmd_loco(args);
  if (group == "msc")       return cmd_msc(args);
  if (group == "state")     return cmd_state(args);
  if (group == "audio")     return cmd_audio(args);
  if (group == "config")    return cmd_config(args);
  if (group == "lowstate")  return cmd_lowstate(args);

  fprintf(stderr, "未知 group: %s\n", group.c_str());
  print_usage();
  return 1;
}
