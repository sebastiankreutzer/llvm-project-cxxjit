//
// Created by sebastian on 20.09.19.
//

#ifndef CLANG_OPTIMIZER_H
#define CLANG_OPTIMIZER_H

#include "clang/CodeGen/BackendUtil.h"
//#include <llvm/IR/LegacyPassManager.h>
#include "llvm/Target/TargetMachine.h"

#include "Optimizer.h"
#include "CodeGenKnobs.h"
#include "Tuners.h"

namespace llvm {
namespace legacy {
class PassManager;
class FunctionPassManager;
} // namespace legacy
} // namespace llvm

namespace clang {
namespace jit {

class StaticOptimizer : public clang::jit::Optimizer {

  static std::once_flag IsPollyInitialized;

  clang::DiagnosticsEngine &Diags;
  const clang::HeaderSearchOptions &HSOpts;
  const clang::CodeGenOptions &CodeGenOpts;
  const clang::TargetOptions &TargetOpts;
  const clang::LangOptions &LangOpts;
  llvm::TargetMachine &TM;

  std::unique_ptr<Tuner> OptTuner;

  OptLvlKnob OptLvl;
  OptSizeKnob OptSizeLvl;

  KnobSet Knobs;

  Module *ModToOptimize;

public:
  StaticOptimizer(clang::DiagnosticsEngine &Diags,
                  const clang::HeaderSearchOptions &HeaderOpts,
                  const clang::CodeGenOptions &CGOpts,
                  const clang::TargetOptions &TOpts, const clang::LangOptions &LOpts,
                  llvm::TargetMachine &TM)
      : Diags(Diags), HSOpts(HeaderOpts), CodeGenOpts(CGOpts),
        TargetOpts(TOpts), LangOpts(LOpts), TM(TM), ModToOptimize(nullptr) {
    Knobs.add(&OptLvl);
    Knobs.add(&OptSizeLvl);

    // OptTuner = std::make_unique<RandomTuner>(Knobs);
    OptTuner = clang::jit::createTuner(clang::jit::loadSearchAlgoEnv(), Knobs);

  }

  void init(llvm::Module *M);

  // Optimize the given module.
  // It is assumed that the module is a (slightly modified) clone of the module
  // that init() was called with.
  ConfigEvalRequest optimize(llvm::Module *M, bool UseDefault);

  const KnobSet &getKnobs() override {
    return Knobs;
  }

private:
  llvm::TargetIRAnalysis getTargetIRAnalysis();

  void createPasses(const llvm::Module &M, llvm::legacy::PassManager &PM,
                    llvm::legacy::FunctionPassManager &FPM, KnobConfig &Cfg);
};

}
}

#endif // CLANG_OPTIMIZER_H
