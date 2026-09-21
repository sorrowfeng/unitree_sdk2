// ============================================================================
// mac_syntax_compat.h
// 仅用于 macOS 上对 unitree SDK 相关源文件做「语法校验」的垫片。
//
// ⚠️ 本文件只服务于 `clang++ -fsyntax-only`：它补齐 Linux 专有类型/常量，
//    使 SDK 头文件在 macOS 上能被解析，从而校验我们自己的代码。
//    它不参与任何构建产物，绝不可用于编译可执行文件（其中 pthread_spinlock_t
//    是用 pthread_mutex_t 顶替的，语义并不等价）。
//
// 用法（见 sim/check_syntax.sh）：
//   clang++ -std=c++17 -fsyntax-only -include sim/shim/mac_syntax_compat.h \
//           -I sim/shim -I include -I thirdparty/include -I example/r1/high_level ...
// ============================================================================
#pragma once

#include <pthread.h>
#include <sched.h>

// macOS 无自旋锁；语法校验只需类型可解析。
typedef pthread_mutex_t pthread_spinlock_t;
static inline int pthread_spin_init(pthread_spinlock_t*, int) { return 0; }
static inline int pthread_spin_destroy(pthread_spinlock_t*) { return 0; }
static inline int pthread_spin_lock(pthread_spinlock_t*) { return 0; }
static inline int pthread_spin_unlock(pthread_spinlock_t*) { return 0; }
static inline int pthread_spin_trylock(pthread_spinlock_t*) { return 1; }

// Linux sched 策略常量在 macOS 缺失（unitree/common/os.hpp 的枚举用到）
#ifndef SCHED_BATCH
#define SCHED_BATCH 3
#endif
#ifndef SCHED_IDLE
#define SCHED_IDLE 5
#endif
#ifndef SCHED_DEADLINE
#define SCHED_DEADLINE 6
#endif
