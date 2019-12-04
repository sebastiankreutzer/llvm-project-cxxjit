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
extern bool EnableDebugFlag;
}
} // namespace clang

#define NOOP                                                                   \
  do {                                                                         \
  } while (false)

#ifdef ENABLE_JIT_DEBUG
#define JIT_DEBUG(X)                                                           \
  do {                                                                         \
    if (clang::jit::EnableDebugFlag)                                           \
      X;                                                                       \
  } while (false)
#define JIT_DEBUG_IF(COND, X)                                                  \
  do {                                                                         \
    if (COND && clang::jit::EnableDebugFlag)                                   \
      X;                                                                       \
  } while (false);
#else
#define JIT_DEBUG(X) NOOP
#define JIT_DEBUG_IF(COND, X) NOOP
#endif

#undef NOOP

#endif // CLANG_JIT_DEBUG_H
