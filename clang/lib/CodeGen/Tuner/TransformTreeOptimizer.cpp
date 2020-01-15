//
// Created by sebastian on 16.12.19.
//

#ifdef POLLY_TUNER
#include "polly/RegisterPasses.h"
#include "polly/Canonicalization.h"
#include "polly/ScopDetection.h"
#endif

#include "clang/CodeGen/BackendUtil.h"
#include "llvm/Transforms/Scalar.h"
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
#include "Util.h"

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
  BackendArgs.push_back("-polly-use-llvm-names");
//  BackendArgs.push_back("--debug-only=polly-ast");
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
  initializeScalarEvolutionWrapperPassPass(*PassRegistry::getPassRegistry());

  legacy::PassManager PM;
  PM.add(createCFGSimplificationPass());
  PM.add(createSROAPass());
  PM.add(createEarlyCSEPass());
  PM.add(createLoopRotatePass());
  PM.add(createIndVarSimplifyPass());
  PM.add(createLoopTransformTreeCreatorPass(LoopTrees));
  PM.run(*M);

  if (LoopTrees.empty()) {
    JIT_INFO(dbgs() << "No loops detected\n");
    return;
  }
  JIT_INFO(dbgs() << "Number of detected loop trees: " << LoopTrees.size() << "\n");


//  CurrentTransformationIdx = 0;
//  Level = 0;

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

  if (UseDefault) {
    if (!BaseLine) {
      BaseLine = Request;
    } else {
      JIT_INFO(errs() << "Warning: evaluating default configs again\n");
    }
  } else {

    if (!DecisionTree) {
      assert(BaseLine && "Baseline must be evaluated before the tree can be created");
      DecisionTree = std::make_unique<TransformDecisionTree>(Tree.clone(), *BaseLine->Stats);
      CurrentNode = &DecisionTree->getRoot();
    }

    assert(CurrentNode && "No node selected");
    if (CurrentNode->isEvaluated()) {

      // Find new transformations based on current best
      CurrentNode->finalize();

      auto Best = CurrentNode->TTuner->getBest();
      assert(Best && "No result");
      auto& BestVal = Best.getValue();

      JIT_INFO(dbgs() << "----------------------------------------\n");
      JIT_INFO(dbgs() << "Transformation fully evaluated - ");

      if (BestVal.Stats->betterThan(*BestNode.second.Stats)) {
        outs() << "New best version found: speedup=" << formatv("{0:f3}\n", computeSpeedup(*BestVal.Stats));
        CurrentNode->printPath(outs()) << "\n";
        BestNode = {CurrentNode, BestVal};
        JIT_INFO(dbgs() << "speedup is " << computeSpeedup(*BestVal.Stats) << " with the following configuration:\n");
        JIT_INFO(KnobState(CurrentNode->TTuner->getKnobs(), BestVal.Cfg).dump());
      } else {
        JIT_INFO(dbgs() << "no speedup\n");
      }
      JIT_INFO(dbgs() << "----------------------------------------\n");

      // Expand the tree
      auto& Promising = DecisionTree->getMostPromisingNode();
      JIT_INFO(dbgs() << "Selected node to expand: kind=" << getTransformationName(Promising.Transformation.Kind) << ", depth=" << Promising.getDepth() << ", nesting_depth=" << Promising.LoopTree->getRoot()->getMaxDepth() <<"\n");
      JIT_INFO(dbgs() << "Available transformations: ");
      unsigned NumAvailable = Promising.FeasibleTransformations.size();
      for (int i = Promising.UnexploredIdx; i < NumAvailable; i++) {
        JIT_INFO(dbgs() << getTransformationName(Promising.FeasibleTransformations[i].Kind) << (i+1 < NumAvailable ? ", " : "\n"));
      }
      auto& NewNode = Promising.expand();
      outs() << "Tuning " << getTransformationName(NewNode.Transformation.Kind) << "\n";
      JIT_INFO(dbgs() << "Expanded Tree:\n");
      // TODO: print tree path
      NewNode.printPath() << "\n";

      CurrentNode = &NewNode;
    }

    Request = CurrentNode->TTuner->getNext();
    JIT_INFO(dbgs() << "Applying transformation on loop " << CurrentNode->Transformation.Root << ":\n");
    JIT_INFO(KnobState(CurrentNode->TTuner->getKnobs(), Request.Cfg).dump());
//    ViewGraph(ClonedTree.get(), "ClonedTree");

    auto TransformedTree = CurrentNode->getTransformedTree(Request.Cfg);
    // Add config values of previous transformations to uniquely identify this code version
    // FIXME: Config may be empty for non-tunable transformations (interchange)
    Request.Cfg.addAll(CurrentNode->FullConfig);

    legacy::PassManager TransformPasses;
    TransformPasses.add(createLoopTransformTreeApplicatorPass({TransformedTree.get()}));

    {
      PrettyStackTraceString CrashInfo("Apply transformation attributes to module");
      TransformPasses.run(*M);
      JIT_DEBUG(util::dumpModule(*M, "With transformation attributes"));
    }


  }


#if 0
  // Old code:

  // Find valid new transformations and reset tuner
  auto resetQueue = [&]() {
    TransformationQueue = findTransformations(&Tree);
    if (TransformationQueue.empty()) {
      JIT_ERROR(errs() << "No valid transformations found");
      llvm_unreachable("implement behavior"); // TODO
    }
    JIT_INFO(dbgs() << "Number of found transformations: " << TransformationQueue.size() << "\n");
    CurrentTransformationIdx = 0;
    BestTransformation = {0, ConfigEvalRequest()};
    TTuner = std::make_unique<TransformationTuner>(TransformationQueue[CurrentTransformationIdx]);
  };

  if (UseDefault) {
    if (!BaseLine) {
      BaseLine = Request;
    } else {
      JIT_INFO(errs() << "Warning: evaluating default configs again\n");
    }
  } else {
    if (TransformationQueue.empty()) {
      resetQueue();
    }
    //  auto& CurrentTransformation = TransformationQueue[CurrentTransformationIdx];

    if (TTuner->isDone()) {
      auto Best = TTuner->getBest();
      assert(Best && "No result");
      auto& BestVal = Best.getValue();

      JIT_INFO(dbgs() << "----------------------------------------\n");
      JIT_INFO(dbgs() << "Transformation fully evaluated - ");

      if (BestVal.Stats->betterThan(*BestTransformation.second.Stats)) {
        BestTransformation = {CurrentTransformationIdx, BestVal};
        JIT_INFO(dbgs() << "speedup is " << computeSpeedup(*BestVal.Stats) << " with the following configuration:\n");
        JIT_INFO(KnobState(TTuner->getKnobs(), BestVal.Cfg).dump());
      } else {
        JIT_INFO(dbgs() << "no speedup\n");
      }
      JIT_INFO(dbgs() << "----------------------------------------\n");

      // All transformations tried, keep the best transformation with the best parameters
      if (++CurrentTransformationIdx >= TransformationQueue.size()) {
        JIT_INFO(dbgs() << "Level " << Level << " transformation tuning complete!\n");
        apply(TransformationQueue[BestTransformation.first], Tree, BestTransformation.second.Cfg);
        FullConfig.addAll(BestTransformation.second.Cfg);
        Level++;
        resetQueue();
      } else {
        TTuner = std::make_unique<TransformationTuner>(TransformationQueue[CurrentTransformationIdx]);
      }
    }

    Request = TTuner->getNext();
    JIT_INFO(dbgs() << "Applying transformation on loop " << TransformationQueue[CurrentTransformationIdx].Root << ":\n");
    JIT_INFO(KnobState(TTuner->getKnobs(), Request.Cfg).dump());
//    ViewGraph(ClonedTree.get(), "ClonedTree");

    auto ClonedTree = Tree.clone();
    apply(TransformationQueue[CurrentTransformationIdx], *ClonedTree, Request.Cfg);
    // Add config values of previous transformations to uniquely identify this code version
    // FIXME: Config may be empty for non-tunable transformations (interchange)
    Request.Cfg.addAll(FullConfig);

    legacy::PassManager TransformPasses;
    TransformPasses.add(createLoopTransformTreeApplicatorPass({ClonedTree.get()}));

    {
      PrettyStackTraceString CrashInfo("Apply transformation attributes to module");
      TransformPasses.run(*M);
      JIT_DEBUG(util::dumpModule(*M, "With transformation attributes"));
    }
  }
#endif


  legacy::PassManager PerModulePasses;
  PerModulePasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  legacy::FunctionPassManager PerFunctionPasses(M);
  PerFunctionPasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  createPasses(*M, PerModulePasses, PerFunctionPasses, Request.Cfg);

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

  if (UseDefault) {
    JIT_INFO(util::dumpModule(*M, "Default optimized module"));
  }

  return Request;

}

float TransformTreeOptimizer::computeSpeedup(TimingStats& Stats) {
  if (!BaseLine)
    return 0;
  auto& BaseLineStats = *BaseLine.getValue().Stats;
  if (!BaseLineStats.Valid())
    return 0;
  if (!Stats.Valid())
    return 0;
  return BaseLineStats.Mean / Stats.Mean;
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
