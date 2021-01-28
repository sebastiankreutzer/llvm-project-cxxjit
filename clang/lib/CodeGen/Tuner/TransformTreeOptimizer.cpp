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
#include "llvm/Support/FormatVariadic.h"


#include "TransformTreeOptimizer.h"
#include "Passes.h"
#include "Debug.h"
#include "LoopTransformTreeTraits.h"
#include "Util.h"
#include "DecisionTreeYAML.h"

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
    Done = true;
    JIT_INFO(outs() << "No loops detected\n");
    return;
  }
  JIT_INFO(outs() << "Number of detected loop trees: " << LoopTrees.size() << "\n");
   // outs() << "Tuner start: " << std::chrono::duration_cast<std::chrono::seconds>(std::chrono::steady_clock::now().time_since_epoch()).count() << "\n";

  CurrentLoopTree = LoopTrees.begin();
  Done = false;


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

void TransformTreeOptimizer::exportDotGraph(LoopTransformTree* Tree, std::string Name)
{
  auto ExportGraphStr = std::getenv("CJ_EXPORT_DOT_GRAPH");
  if (!ExportGraphStr)
    return;
  auto Filename = Name + "_" + ExportGraphStr + ".dot";
  WriteGraph(Tree, Name, false, Name, Filename);
}

void TransformTreeOptimizer::exportTree() {
  auto ExportTreeStr = std::getenv("CJ_EXPORT_TREE");
  if (!ExportTreeStr)
    return;

  auto FileEnv = std::getenv("CJ_TREE_FILE_SUFFIX");
  std::string Filename;
  if (FileEnv) {
    Filename = (SearchTree->getRoot().LoopTree->getName() + "_" + FileEnv + ".yaml").str();
  } else {
    Filename = (SearchTree->getRoot().LoopTree->getName() + ".yaml").str();
  }

  writeTree(*SearchTree, Filename);
}

ConfigEval TransformTreeOptimizer::optimize(llvm::Module *M, bool UseDefault) {
  assert(ModToOptimize && "Optimizer is not initialized!");

  setCommandLineOpts(CodeGenOpts);

  ConfigEval Request;

  auto buildSearchTree = [&](LoopTransformTree& Base) {
    assert(BaseLine && "Baseline must be evaluated before the tree can be created");
    SearchTree = std::make_unique<TransformSearchTree>(Base.clone(), *BaseLine->Stats);
    CurrentNode = &SearchTree->getRoot();
    BestNode = {};
    RestartCounts.clear();
  };


    if (UseDefault) {
      if (!BaseLine) {
        BaseLine = Request;
      } else {
        JIT_INFO(errs() << "Warning: evaluating default configs again\n");
      }
    } else {
      if (!Done) {

        auto &Tree = **CurrentLoopTree;

        if (!SearchTree) {
          buildSearchTree(Tree);
        }

        assert(CurrentNode && "No node selected");
        if (CurrentNode->isEvaluated()) {

          // Find new transformations based on current best
          CurrentNode->finalize();

//          if (CurrentNode->Transformation.Kind == LoopTransformation::INTERCHANGE) {
//            if (auto Parent = CurrentNode->Parent; Parent && Parent->Transformation.Kind == LoopTransformation::UNROLL_AND_JAM) {
//              if (auto Grandparent = Parent->Parent; Grandparent && Grandparent->Transformation.Kind == LoopTransformation::TILE) {
//                ViewGraph<LoopTransformTree*>(Parent->TransformedTree.get(), "transformation tree", false, "transformation tree after unroll-and-jam");
//                ViewGraph<LoopTransformTree*>(CurrentNode->TransformedTree.get(), "transformation tree", false, "transformation tree after interchange");
//                JIT_INFO(outs() << "Root is: " << CurrentNode->TransformedTree->getRoot()->getLoopName()
//                                << ", last successor of root is "
//                                << CurrentNode->TransformedTree->getRoot()->getLastSuccessor()->getLoopName()
//                                << ", effective root is "
//                                << CurrentNode->TransformedTree->getEffectiveSuccessorRoot()->getLoopName() << "\n");
//                exit(0);
//              }
//            }
//          }

          exportTree();

          unsigned RestartIndex = CurrentNode->TTuner->getLastImprovementRestartIndex();
          JIT_INFO(outs() << "Search needed " << RestartIndex << " restarts to find the best version.\n");
          if (RestartIndex >= RestartCounts.size()) {
            RestartCounts.resize(RestartIndex+1, 0);
          }
          RestartCounts[RestartIndex]++;

          auto Best = CurrentNode->TTuner->getBest();
          assert(Best && "No result");
          auto &BestVal = Best.getValue();

          JIT_INFO(outs() << "----------------------------------------\n");
          JIT_INFO(outs() << "Transformation fully evaluated with " << CurrentNode->TTuner->getNumConfigs() << " configs - ");

          float NodeSpeedup = computeSpeedup(*BestVal.Stats);

          if (BestVal.Stats->betterThan(*BestNode.second.Stats)) {
            JIT_INFO(outs() << "New best version found: speedup is " << formatv("{0:f3}\n", NodeSpeedup));
            JIT_INFO(CurrentNode->printPath(outs()) << "\n");
            BestNode = {CurrentNode, BestVal};
          } else {
            JIT_INFO(outs() << "speedup is " << formatv("{0:f3}\n", NodeSpeedup));
          }
          JIT_INFO(outs() << "----------------------------------------\n");

          if (SearchTree->isFullyExplored()) {

            // Tuning is done, save best configuration
            auto FinalTree = BestNode.first->applyBestConfig();

            //ViewGraph<LoopTransformTree*>(FinalTree.get(), "Final transformation tree", false, "Final transformation tree");

            exportTree();
            exportDotGraph(FinalTree.get(), SearchTree->getRoot().LoopTree->getName());


              JIT_INFO(outs() << "Loop nest fully explored!\n");
            JIT_INFO(outs() << "Best transformation sequence: ");
            JIT_INFO(BestNode.first->printPath(outs()) << "\n");

            JIT_INFO(outs() << "Restart Stats:\n");
            unsigned NumSearches = 0;
            for (auto C : RestartCounts) {
              NumSearches += C;
            }
            for (unsigned I = 0; I < RestartCounts.size(); I++) {
              JIT_INFO(outs() << I << " restarts needed: " << RestartCounts[I] << formatv("({0:p})\n", static_cast<float>(RestartCounts[I]) / NumSearches));
            }

            FinalizedTrees.push_back(std::move(FinalTree));
            CurrentLoopTree++;

            if (CurrentLoopTree == LoopTrees.end()) {
              Done = true;
              outs() << "Autotuning complete!\n";
            } else {
              // Build decision for next loop nest
              buildSearchTree(**CurrentLoopTree);
            }

          } else {
            // Expand the tree
            auto &Promising = SearchTree->getMostPromisingNode(AllowRegression);
            JIT_INFO(outs() << "Selected node to expand: kind="
                            << (Promising.getDepth() == 1 ? "ROOT" : getTransformationName(Promising.Transformation.Kind))
                            << ", depth=" << Promising.getDepth() << ", nesting_depth="
                            << Promising.LoopTree->getRoot()->getMaxDepth() << "\n");
            JIT_INFO(outs() << "Available transformations: ");
            unsigned NumAvailable = Promising.FeasibleTransformations.size();
            for (int i = Promising.UnexploredIdx; i < NumAvailable; i++) {
              JIT_INFO(outs() << getTransformationName(Promising.FeasibleTransformations[i].Kind)
                              << (i + 1 < NumAvailable ? ", " : "\n"));
            }
            auto &NewNode = Promising.expand();
            NewNode.ExpansionID = ExpansionCounter++;
            JIT_INFO(outs() << "Expanded Tree:\n");
            JIT_INFO(NewNode.printPath() << "\n");


            CurrentNode = &NewNode;
          }

        }
      }

      SmallVector<LoopTransformTree *, 2> ToApply;
      LoopTransformTreePtr TransformedTree;

      // Add pass to transform currently investigated loop nest
      // Note: Done is checked here again because it can change during the update.
      if (!Done) {

        Request = CurrentNode->TTuner->getNext();

        //    ViewGraph(ClonedTree.get(), "ClonedTree");

        TransformedTree = CurrentNode->getTransformedTree(Request.Config);

        ToApply.push_back(TransformedTree.get());
      }

    // Add passes to apply the found configuration for finalized loop nests
    for (auto& Tree : FinalizedTrees) {
      ToApply.push_back(Tree.get());
    }

//    Request.Cfg.addAll(FinalizedConfig);

    legacy::PassManager TransformPasses;
    TransformPasses.add(createLoopTransformTreeApplicatorPass(ToApply));

    {
      PrettyStackTraceString CrashInfo("Apply transformation attributes to module");
      TransformPasses.run(*M);
      JIT_DEBUG(util::dumpModule(*M, "With transformation attributes"));
    }

  }


  legacy::PassManager PerModulePasses;
  PerModulePasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  legacy::FunctionPassManager PerFunctionPasses(M);
  PerFunctionPasses.add(
      createTargetTransformInfoWrapperPass(getTargetIRAnalysis()));

  createPasses(*M, PerModulePasses, PerFunctionPasses, Request.Config, UseDefault);

  // Before executing passes, print the final values of the LLVM options.
  cl::PrintOptionValues();

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
    JIT_DEBUG(util::dumpModule(*M, "Default optimized module"));
  }

  return Request;

}

float TransformTreeOptimizer::computeSpeedup(TimingStats& Stats) {
  if (!BaseLine)
    return 0;
  auto& BaseLineStats = *BaseLine.getValue().Stats;
  if (!BaseLineStats.valid())
    return 0;
  if (!Stats.valid())
    return 0;
  return BaseLineStats.Mean / Stats.Mean;
}

TargetIRAnalysis TransformTreeOptimizer::getTargetIRAnalysis() {
  return TM.getTargetIRAnalysis();
}

void TransformTreeOptimizer::createPasses(const llvm::Module &M, legacy::PassManager &PM,
                                   legacy::FunctionPassManager &FPM,
                                   ParamConfig &Cfg, bool DefaultOpt) {

  auto OptLevel = 3;     // OptLvl.getVal(Cfg);
  auto OptSizeLevel = 0; // OptSizeLvl.getVal(Cfg); // FIXME TODO

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
  bool VectorizeLoop = CodeGenOpts.VectorizeLoop;
  bool Unroll = CodeGenOpts.UnrollLoops;
  bool Reroll = CodeGenOpts.RerollLoops;

  if (!DefaultOpt) {
    if (!VectorizeSLP) {
      //JIT_INFO(errs() << "VectorizeSLP is disabled - enabling for tuning.\n");
      VectorizeSLP = true;
    }

    if (!VectorizeLoop) {
      //JIT_INFO(errs() << "VectorizeLoop is disabled - enabling for tuning.\n");
      VectorizeLoop = true;
    }

    if (Unroll || Reroll) {
      //JIT_INFO(errs() << "Unrolling/rerolling is enabled - disabling for tuning\n");
      Unroll = false;
      Reroll = false;
    }
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
