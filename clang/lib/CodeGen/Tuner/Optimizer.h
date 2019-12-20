//
// Created by sebastian on 16.12.19.
//

#ifndef LLVM_OPTIMIZER_H
#define LLVM_OPTIMIZER_H

#include "Tuner.h"

namespace clang {
namespace jit {


class Optimizer {
public:

//  static std::once_flag IsPollyInitialized;


  virtual void init(llvm::Module* M) = 0;

  // Optimize the given module.
  // It is assumed that the module is a (slightly modified) clone of the module
  // that init() was called with.
  virtual ConfigEvalRequest optimize(llvm::Module *M, bool UseDefault) = 0;

  virtual const KnobSet& getKnobs() = 0;

};

}
}

#endif //LLVM_OPTIMIZER_H
