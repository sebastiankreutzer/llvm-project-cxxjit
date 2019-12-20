//
// Created by sebastian on 16.12.19.
//

#ifdef POLLY_TUNER
#include "polly/RegisterPasses.h"
#include "polly/Canonicalization.h"
#include "polly/ScopDetection.h"
#endif

#include "clang/CodeGen/BackendUtil.h"
#include "llvm/Transforms/IPO/PassManagerBuilder.h"
#include "llvm/Transforms/IPO.h"
#include "llvm/Analysis/TargetLibraryInfo.h"
#include "llvm/Analysis/TargetTransformInfo.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Support/PrettyStackTrace.h"
#include "llvm/Support/GraphWriter.h"
#include "llvm/IR/IRPrintingPasses.h"


#include "TransformTreeOptimizer.h"
#include "Passes.h"
#include "Debug.h"
#include "LoopTransformTreeTraits.h"

using namespace llvm;

namespace clang {

namespace jit {

std::once_flag TransformTreeOptimizer::IsPollyInitialized;

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


void TransformTreeOptimizer::init(Module *M) {
  this->ModToOptimize = M;

  assert(POLLY_TUNER && "This optimizer cannot be used without polly!");
  // TODO: A better idea is to only compile this class if polly is enabled. Check for POLLY_TUNER somewhere in driver where the object is created.

#ifdef POLLY_TUNER

  std::call_once(IsPollyInitialized, [] {
    JIT_INFO(outs() << "Initializing polly...\n");
    polly::PollyProcessUnprofitable = true;
    // polly::PollyInvariantLoadHoisting = true; // TODO: should this be on?

    llvm::PassRegistry &Registry = *llvm::PassRegistry::getPassRegistry();
    polly::initializePollyPasses(Registry);
  });

#endif

  // Create loop knobs
  initializeLoopInfoWrapperPassPass(*PassRegistry::getPassRegistry());
  legacy::PassManager PM;
  PM.add(createLoopTransformTreeCreatorPass(LoopTrees));
  PM.run(*M);

  if (LoopTrees.empty()) {
    JIT_INFO(dbgs() << "No loops detected\n");
    return;
  }
  JIT_INFO(dbgs() << "Number of detected loop trees: " << LoopTrees.size() << "\n");


//  for (auto It = Tree.nodes_begin();  It != Tree.nodes_end(); ++It) {
//    errs() << "Node: " << (*It)->getLoopName() << "\n";
//  }

//  JIT_INFO(errs() << "Tree contains " << Tree.size() << " individual loops:\n");
//  for (auto It = GraphTraits<LoopTransformTree>::nodes_begin(&Tree); It != GraphTraits<LoopTransformTree>::nodes_end(&Tree); ++It) {
//    auto* Node = *It;
//    JIT_INFO(errs() << Node->getLoopName() << ": ");
//    for (auto* Child : Node->subLoops()) {
//      JIT_INFO(errs() << Child->getLoopName() << ", ");
//    }
//    JIT_INFO(errs() << "Again:\n");
//    for (auto It = GraphTraits<LoopTransformTree>::child_begin(Node); It != GraphTraits<LoopTransformTree>::child_end(Node); ++It) {
//      JIT_INFO(errs() << (*It)->getLoopName() << ", ");
//    }
//    JIT_INFO(errs() << "\n");
//  }

  //WriteGraph(&Tree, "Initial Loop Tree", false, "Initial Loop Tree", "./jit_graph.dot");
//  ViewGraph(&Tree, "Initial Loop Tree");
//
//  JIT_INFO(errs() << "Applying tiling to loops...\n");
//
//
//  applyTiling(&Tree);
//
//  ViewGraph(&Tree, "After tiling");
}

ConfigEvalRequest TransformTreeOptimizer::optimize(llvm::Module *M, bool UseDefault) {
  assert(ModToOptimize && "Optimizer is not initialized!");

  setCommandLineOpts(CodeGenOpts);

  ConfigEvalRequest Request;

//  if (UseDefault) {
//    JIT_DEBUG(dbgs() << "Using default optimization config\n");
//    auto Cfg = createDefaultConfig(Knobs);
//    setEnableLoopTransform(Cfg, false);
//    Request = ConfigEvalRequest(Cfg);
//  } else {
//    Request = OptTuner->generateNextConfig();
//  }
//
   auto &Cfg = Request.Cfg;
//
//  KnobState KS(Knobs, Cfg);
//  JIT_DEBUG(dbgs() << "Optimizer Configuration: "
//                   << "\n");
//  JIT_DEBUG(dbgs() << "-------------------- "
//                   << "\n");
//  JIT_DEBUG(KS.dump());
//  JIT_DEBUG(dbgs() << "-------------------- "
//                   << "\n");

  // TODO: This is just here for testing, eventually the tree should be transformed in optimize()
  if (LoopTrees.size() > 1) {
    JIT_INFO(errs() << "Optimized function has multiple loop trees - optimizing only the first.\n");
  }

  auto& Tree = *LoopTrees.front();

  auto ClonedTree = Tree.clone();

  if (!UseDefault) {
    applyTiling(ClonedTree.get());
  }

//  ViewGraph(ClonedTree.get(), "After tiling");


  legacy::PassManager TransformPasses;
  TransformPasses.add(createLoopTransformTreeApplicatorPass({ClonedTree.get()}));

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
    TransformPasses.run(*M);
#ifdef DUMP_MOD_WITH_ATTRIBUTES
    outs() << "*****************************\n";
    outs() << "With transform attributes\n";
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

TargetIRAnalysis TransformTreeOptimizer::getTargetIRAnalysis() {
  return TM.getTargetIRAnalysis();
}

void TransformTreeOptimizer::createPasses(const llvm::Module &M, legacy::PassManager &PM,
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

  bool VectorizeSLP = CodeGenOpts.VectorizeSLP;
  if (!VectorizeSLP) {
    JIT_INFO(errs() << "VectorizeSLP is disabled - enabling for tuning.\n");
    VectorizeSLP = true;
  }

  bool VectorizeLoop = CodeGenOpts.VectorizeLoop;
  if (!VectorizeLoop) {
    JIT_INFO(errs() << "VectorizeLoop is disabled - enabling for tuning.\n");
    VectorizeLoop = true;
  }

  bool Unroll = CodeGenOpts.UnrollLoops;
  bool Reroll = CodeGenOpts.RerollLoops;
  if (Unroll || Reroll) {
    JIT_INFO(errs() << "Unrolling/rerolling is enabled - disabling for tuning\n");
    Unroll = false;
    Reroll = false;
  }


  PMB.OptLevel = OptLevel;
  PMB.SizeLevel = OptSizeLevel;
  PMB.SLPVectorize = VectorizeSLP;
  PMB.LoopVectorize = VectorizeLoop;

  PMB.DisableUnrollLoops = !Unroll;
  PMB.RerollLoops = Reroll;
  PMB.MergeFunctions = CodeGenOpts.MergeFunctions;
  PMB.PrepareForThinLTO = CodeGenOpts.PrepareForThinLTO; // TODO: behavior for jit?
  PMB.PrepareForLTO = CodeGenOpts.PrepareForLTO;

  TM.adjustPassManager(PMB);

  PM.add(new TargetLibraryInfoWrapperPass(*TLII));
  FPM.add(new TargetLibraryInfoWrapperPass(*TLII));


#ifdef POLLY_TUNER

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

}
}
