//
// Created by sebastian on 20.09.19.
//

#ifndef CLANG_OPTIMIZER_H
#define CLANG_OPTIMIZER_H

#include "clang/CodeGen/BackendUtil.h"
//#include <llvm/IR/LegacyPassManager.h>
#include "llvm/Target/TargetMachine.h"

#include "Tuner.h"
#include "CodeGenKnobs.h"

namespace llvm {
  namespace legacy {
  class PassManager;
  class FunctionPassManager;
  }
}

namespace tuner {

class Optimizer {

  clang::DiagnosticsEngine &Diags;
  const clang::HeaderSearchOptions &HSOpts;
  const clang::CodeGenOptions &CodeGenOpts;
  const clang::TargetOptions &TargetOpts;
  const clang::LangOptions &LangOpts;
  llvm::TargetMachine& TM;

  std::unique_ptr<Tuner> OptTuner;

  OptLvlKnob OptLvl;
  OptSizeKnob OptSizeLvl;

  KnobSet Knobs;

public:
  Optimizer(clang::DiagnosticsEngine &Diags,
            const clang::HeaderSearchOptions &HeaderOpts,
            const clang::CodeGenOptions &CGOpts,
            const clang::TargetOptions &TOpts,
            const clang::LangOptions &LOpts,
            llvm::TargetMachine& TM)
            : Diags(Diags), HSOpts(HeaderOpts), CodeGenOpts(CGOpts), TargetOpts(TOpts), LangOpts(LOpts), TM(TM) {
    Knobs.add(&OptLvl);
    Knobs.add(&OptSizeLvl);
    OptTuner = llvm::make_unique<RandomTuner>(Knobs);
  }

  void optimize(llvm::Module* M);


private:
  llvm::TargetIRAnalysis getTargetIRAnalysis();

  void createPasses(const llvm::Module& M, llvm::legacy::PassManager& PM, llvm::legacy::FunctionPassManager& FPM, KnobConfig& Cfg);

};

}


#endif //CLANG_OPTIMIZER_H
