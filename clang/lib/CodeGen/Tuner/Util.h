//
// Created by sebastian on 25.09.19.
//

#ifndef CLANG_UTIL_H
#define CLANG_UTIL_H

#include "llvm/Support/raw_ostream.h"
#include "llvm/Support/Debug.h"

#include "llvm/IR/Module.h"
#include <chrono>

namespace clang {
namespace jit {

namespace util {

inline unsigned genSeed() {
  return static_cast<unsigned>(
      std::chrono::system_clock::now().time_since_epoch().count());
}

/**
 * Check if the environment variable with the given name is set to the expected value.
 *
 * @param Name
 * @param Expected If empty, checks if any value is set.
 * @return
 */
inline bool checkEnv(const char* Name, const char* Expected) {
  auto EnvStr = std::getenv(Name);
  if (EnvStr) {
    if (!Expected || !Expected[0]) {
      return true;
    }
    if (std::strcmp(EnvStr, Expected) == 0)
      return true;
  }
  return false;
}

inline void dumpModule(const llvm::Module& M, llvm::StringRef Status) {
  llvm::dbgs() << "*****************************\n";
  llvm::dbgs() << Status << "\n";
  llvm::dbgs() << "*****************************\n";
  M.dump();
  llvm::dbgs().flush();
}


} // namespace util

}
}

#endif // CLANG_UTIL_H
