//
// Created by sebastian on 20.09.19.
//

#include "StaticOptimizer.h"

#include "Passes.h"
#include "Debug.h"

#ifdef POLLY_TUNER
#include "polly/RegisterPasses.h"
#include "polly/Canonicalization.h"
#include "polly/ScopDetection.h"
#endif

#include "clang/Basic/CodeGenOptions.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/IR/IRPrintingPasses.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"

#include "llvm/Transforms/Utils/Cloning.h"


using namespace clang;
using namespace llvm;

namespace clang {
namespace jit {


std::once_flag StaticOptimizer::IsPollyInitialized;

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
  llvm::cl::ParseCommandLineOptions(BackendArgs.size() - 1, BackendArgs.data());
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

TargetIRAnalysis StaticOptimizer::getTargetIRAnalysis() {
  return TM.getTargetIRAnalysis();
}

void StaticOptimizer::createPasses(const llvm::Module &M, legacy::PassManager &PM,
                                   legacy::FunctionPassManager &FPM,
                                   KnobConfig &Cfg) {
  auto OptLevel = 3;     // OptLvl.getVal(Cfg);
  auto OptSizeLevel = 1; // OptSizeLvl.getVal(Cfg); // FIXME TODO

  PassManagerBuilder PMB;

  // Figure out TargetLibraryInfo.  This needs to be added to MPM and FPM
  // manually (and not via PMBuilder), since some passes (eg. InstrProfiling)
  // are inserted before PMBuilder ones - they'd get the default-constructed
  // TLI with an unknown target otherwise.
  Triple TargetTriple(M.getTargetTriple());
  std::unique_ptr<TargetLibraryInfoImpl> TLII(
      createTLII(TargetTriple, CodeGenOpts));

  // Clang uses AlwaysInlinerPass for -00 and -O1 for speed.
  // As we want to set the inlining threshold independently anyway, we always
  // use the "default" inlinig pass.
  // TODO: Not sure about DisableInlineHotCallSite, see BackendUtil.cpp
  PMB.Inliner =
      createFunctionInliningPass(OptLevel, OptSizeLevel,
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


#ifdef POLLY_TUNER
  JIT_DEBUG(dbgs() << "Initializing polly passes...\n");

  std::call_once(IsPollyInitialized, [] {
    polly::PollyProcessUnprofitable = true;
    // polly::PollyInvariantLoadHoisting = true; // TODO: should this be on?

    llvm::PassRegistry &Registry = *llvm::PassRegistry::getPassRegistry();
    polly::initializePollyPasses(Registry);
  });

  // Run polly before other optimizations
  // TOOD: Investigate best time to apply polly transformations. For example, the inliner may need to run before polly to allow vectorization.
  PMB.addExtension(llvm::PassManagerBuilder::EP_ModuleOptimizerEarly,
                   [=](PassManagerBuilder const &Builder, PassManagerBase &PM) {
                     JIT_DEBUG(PM.add(
                         llvm::createPrintModulePass(llvm::dbgs(), "\n;------------\n\n\n\n\n; Before Polly\n")));

                     polly::registerCanonicalicationPasses(PM);
                     polly::registerPollyPasses(PM);

                     JIT_DEBUG(PM.add(llvm::createPrintModulePass(llvm::dbgs(), "\n\n\n\n\n; After Polly\n")));
                   });
#endif
  PMB.populateModulePassManager(PM);
  PMB.populateFunctionPassManager(FPM);

}


void StaticOptimizer::init(Module *M) {
  this->ModToOptimize = M;

  // Create loop knobs
  initializeLoopInfoWrapperPassPass(*PassRegistry::getPassRegistry());
  legacy::PassManager PM;
  PM.add(createLoopKnobCreatorPass(Knobs));
  PM.run(*M);
  OptTuner->reset(Knobs);
  JIT_INFO(dbgs() << "Search space dimension: " << Knobs.countTunable() << "\n");

}

//#define DUMP_MOD_WITH_ATTRIBUTES
ConfigEvalRequest StaticOptimizer::optimize(Module *M, bool UseDefault) {
  assert(ModToOptimize && "Optimizer is not initialized!");

  setCommandLineOpts(CodeGenOpts);

  ConfigEvalRequest Request;

  if (UseDefault) {
    JIT_DEBUG(dbgs() << "Using default optimization config\n");
    auto Cfg = createDefaultConfig(Knobs);
    setEnableLoopTransform(Cfg, false);
    Request = ConfigEvalRequest(Cfg);
  } else {
    Request = OptTuner->generateNextConfig();
  }

  auto &Cfg = Request.Cfg;

  KnobState KS(Knobs, Cfg);
  JIT_DEBUG(dbgs() << "Optimizer Configuration: "
                   << "\n");
  JIT_DEBUG(dbgs() << "-------------------- "
                   << "\n");
  JIT_DEBUG(KS.dump());
  JIT_DEBUG(dbgs() << "-------------------- "
                   << "\n");

  legacy::PassManager KnobPasses;
  KnobPasses.add(createApplyLoopKnobPass(Cfg));

  legacy::PassManager PerModulePasses;
  PerModulePasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  legacy::FunctionPassManager PerFunctionPasses(M);
  PerFunctionPasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  createPasses(*M, PerModulePasses, PerFunctionPasses, Cfg);

  // Before executing passes, print the final values of the LLVM options.
  cl::PrintOptionValues();

  // Run passes. For now we do all passes at once, but eventually we
  // would like to have the option of streaming code generation.

  {
    PrettyStackTraceString CrashInfo("Apply knobs to module");
    KnobPasses.run(*M);
#ifdef DUMP_MOD_WITH_ATTRIBUTES
    outs() << "*****************************\n";
    outs() << "After loop knob application\n";
    outs() << "*****************************\n";
    outs().flush();
    M->dump();
    errs().flush();
#endif
  }

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

  return Request;
}

}
}