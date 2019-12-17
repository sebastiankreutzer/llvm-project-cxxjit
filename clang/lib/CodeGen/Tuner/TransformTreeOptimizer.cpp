//
// Created by sebastian on 16.12.19.
//

#include "llvm/IR/LegacyPassManager.h"

#include "TransformTreeOptimizer.h"
#include "Passes.h"
#include "Debug.h"

using namespace llvm;

namespace clang {

namespace jit {

void TransformTreeOptimizer::init(Module *M) {
  this->ModToOptimize = M;

  // Create loop knobs
  initializeLoopInfoWrapperPassPass(*PassRegistry::getPassRegistry());
  legacy::PassManager PM;
  PM.add(createLoopTransformTreeCreatorPass(LoopTrees));
  PM.run(*M);
}

ConfigEvalRequest TransformTreeOptimizer::optimize(llvm::Module *M, bool UseDefault) {

    if (LoopTrees.size() > 1) {
      JIT_INFO(errs() << "Optimized function has multiple loop trees - optimizing only the first.\n")
    }


}

}

}
