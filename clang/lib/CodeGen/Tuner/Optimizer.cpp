//
// Created by sebastian on 20.09.19.
//

#include "Optimizer.h"

#include "clang/Basic/CodeGenOptions.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/Transforms/IPO.h"


using namespace clang;
using namespace llvm;

namespace tuner {

//
// Copied from BackendUtil.cpp
//

static void setCommandLineOpts(const CodeGenOptions &CodeGenOpts) {
  SmallVector<const char *, 16> BackendArgs;
  BackendArgs.push_back("clang"); // Fake program name.
  if (!CodeGenOpts.DebugPass.empty()) {
    BackendArgs.push_back("-debug-pass");
    BackendArgs.push_back(CodeGenOpts.DebugPass.c_str());
  }
  if (!CodeGenOpts.LimitFloatPrecision.empty()) {
    BackendArgs.push_back("-limit-float-precision");
    BackendArgs.push_back(CodeGenOpts.LimitFloatPrecision.c_str());
  }
  BackendArgs.push_back(nullptr);
  llvm::cl::ParseCommandLineOptions(BackendArgs.size() - 1,
                                    BackendArgs.data());
}

static TargetLibraryInfoImpl *createTLII(llvm::Triple &TargetTriple,
                                         const CodeGenOptions &CodeGenOpts) {
  TargetLibraryInfoImpl *TLII = new TargetLibraryInfoImpl(TargetTriple);
  if (!CodeGenOpts.SimplifyLibCalls)
    TLII->disableAllFunctions();
  else {
    // Disable individual libc/libm calls in TargetLibraryInfo.
    LibFunc F;
    for (auto &FuncName : CodeGenOpts.getNoBuiltinFuncs())
      if (TLII->getLibFunc(FuncName, F))
        TLII->setUnavailable(F);
  }

  switch (CodeGenOpts.getVecLib()) {
    case CodeGenOptions::Accelerate:
      TLII->addVectorizableFunctionsFromVecLib(TargetLibraryInfoImpl::Accelerate);
      break;
    case CodeGenOptions::SVML:
      TLII->addVectorizableFunctionsFromVecLib(TargetLibraryInfoImpl::SVML);
      break;
    default:
      break;
  }
  return TLII;
}

//
//
//

TargetIRAnalysis Optimizer::getTargetIRAnalysis() {
    return TM.getTargetIRAnalysis();
}

void Optimizer::createPasses(const llvm::Module& M, legacy::PassManager &PM, legacy::FunctionPassManager &FPM, KnobConfig& Cfg) {
  auto OptLevel = OptLvl.getVal(Cfg);
  auto OptSizeLevel = OptSizeLvl.getVal(Cfg);

  PassManagerBuilder PMB;

  // Figure out TargetLibraryInfo.  This needs to be added to MPM and FPM
  // manually (and not via PMBuilder), since some passes (eg. InstrProfiling)
  // are inserted before PMBuilder ones - they'd get the default-constructed
  // TLI with an unknown target otherwise.
  Triple TargetTriple(M.getTargetTriple());
  std::unique_ptr<TargetLibraryInfoImpl> TLII(
      createTLII(TargetTriple, CodeGenOpts));

  // Clang uses AlwaysInlinerPass for -00 and -O1 for speed.
  // As we want to set the inlining threshold independently anyway, we always use the "default" inlinig pass.
  // TODO: Not sure about DisableInlineHotCallSite, see BackendUtil.cpp
  PMB.Inliner = createFunctionInliningPass(
      OptLevel, OptSizeLevel,
      (!CodeGenOpts.SampleProfileFile.empty() &&
       CodeGenOpts.PrepareForThinLTO));


  PMB.OptLevel = OptLevel;
  PMB.SizeLevel = OptSizeLevel;
  PMB.SLPVectorize = CodeGenOpts.VectorizeSLP;
  PMB.LoopVectorize = CodeGenOpts.VectorizeLoop;

  PMB.DisableUnrollLoops = !CodeGenOpts.UnrollLoops;
  PMB.MergeFunctions = CodeGenOpts.MergeFunctions;
  PMB.PrepareForThinLTO = CodeGenOpts.PrepareForThinLTO;
  PMB.PrepareForLTO = CodeGenOpts.PrepareForLTO;
  PMB.RerollLoops = CodeGenOpts.RerollLoops;

  TM.adjustPassManager(PMB);

  PM.add(new TargetLibraryInfoWrapperPass(*TLII));
  FPM.add(new TargetLibraryInfoWrapperPass(*TLII));

  PMB.populateModulePassManager(PM);
  PMB.populateFunctionPassManager(FPM);

}

void Optimizer::optimize(Module* M) {
  setCommandLineOpts(CodeGenOpts);

  legacy::PassManager PerModulePasses;
  PerModulePasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  legacy::FunctionPassManager PerFunctionPasses(M);
  PerFunctionPasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  // TODO: For now, generate new config everytime optimize() is called. Figure out where that decision is to be made.
  KnobConfig Cfg = OptTuner->generateNextConfig();

  KnobState KS(Knobs, Cfg);
  outs() << "Tuner Configuration: " << "\n";
  KS.dump();

  createPasses(*M, PerModulePasses, PerFunctionPasses, Cfg);

  // Before executing passes, print the final values of the LLVM options.
  cl::PrintOptionValues();

  // Run passes. For now we do all passes at once, but eventually we
  // would like to have the option of streaming code generation.

  {
    PrettyStackTraceString CrashInfo("Per-function optimization");

    PerFunctionPasses.doInitialization();
    for (Function &F : *M)
      if (!F.isDeclaration())
        PerFunctionPasses.run(F);
    PerFunctionPasses.doFinalization();
  }

  {
    PrettyStackTraceString CrashInfo("Per-module optimization passes");
    PerModulePasses.run(*M);
  }

}

}