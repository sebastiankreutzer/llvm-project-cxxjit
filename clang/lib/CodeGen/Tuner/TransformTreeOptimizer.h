//
// Created by sebastian on 16.12.19.
//

#ifndef LLVM_TRANSFORMTREESEARCH_H
#define LLVM_TRANSFORMTREESEARCH_H

#include "clang/Basic/Diagnostic.h"
#include "clang/Lex/HeaderSearchOptions.h"
#include "clang/Basic/CodeGenOptions.h"
#include "clang/Basic/TargetOptions.h"
#include "llvm/Target/TargetMachine.h"

#include "Optimizer.h"
#include "LoopTransformTree.h"

using namespace llvm;

namespace clang {
namespace jit {


class TransformTreeOptimizer: public Optimizer {
public:

  TransformTreeOptimizer(clang::DiagnosticsEngine &Diags,
                         const clang::HeaderSearchOptions &HeaderOpts,
                         const clang::CodeGenOptions &CGOpts,
                         const clang::TargetOptions &TOpts, const clang::LangOptions &LOpts,
                         llvm::TargetMachine &TM)
      : Diags(Diags), HSOpts(HeaderOpts), CodeGenOpts(CGOpts),
        TargetOpts(TOpts), LangOpts(LOpts), TM(TM), ModToOptimize(nullptr) {
  }

  void init(Module* M) override;

  ConfigEvalRequest optimize(llvm::Module *M, bool UseDefault) override;

private:
  clang::DiagnosticsEngine &Diags;
  const clang::HeaderSearchOptions &HSOpts;
  const clang::CodeGenOptions &CodeGenOpts;
  const clang::TargetOptions &TargetOpts;
  const clang::LangOptions &LangOpts;
  llvm::TargetMachine &TM;
  Module *ModToOptimize;
  SmallVector<LoopTransformTree, 2> LoopTrees;
};


}
}


#endif //LLVM_TRANSFORMTREESEARCH_H
