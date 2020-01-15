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
