//
// Created by sebastian on 03.12.19.
//

#ifndef CLANG_JIT_DEBUG_H
#define CLANG_JIT_DEBUG_H

// NOTE: Using LLVM's debug macro does not really work at run time

// TODO: CMake
#define ENABLE_JIT_DEBUG

namespace clang {
namespace jit {

enum LogLevel {
  LOG_ERROR = 0,
  LOG_INFO = 1,
  LOG_REPORT = 2,
  LOG_DEBUG = 3
};

extern unsigned LogLvl;
}
} // namespace clang

#define NOOP                                                                   \
  do {                                                                         \
  } while (false)

#ifdef ENABLE_JIT_DEBUG

#define JIT_LOG(LVL, X)                                                           \
  do {                                                                         \
    if (LVL <= clang::jit::LogLvl)                                           \
      X;                                                                       \
  } while (false)
#define JIT_INFO(X) JIT_LOG(clang::jit::LOG_INFO, X)
#define JIT_REPORT(X) JIT_LOG(clang::jit::LOG_REPORT, X)
#define JIT_DEBUG(X) JIT_LOG(clang::jit::LOG_DEBUG, X)
#define JIT_ERROR(X) JIT_LOG(clang::jit::LOG_ERROR, X)
#define JIT_LOG_IF(COND, LVL, X)                                                  \
  do {                                                                         \
    if (COND && LVL <= clang::jit::LogLvl)                                   \
      X;                                                                       \
  } while (false);
#else
#define JIT_LOG(LVL, X) NOOP
#define JIT_INFO(X) NOOP
#define JIT_REPORT(X) NOOP
#define JIT_DEBUG(X) NOOP
#define JIT_LOG_IF(COND, LVL, X)    NOOP
#endif

#undef NOOP

#endif // CLANG_JIT_DEBUG_H
