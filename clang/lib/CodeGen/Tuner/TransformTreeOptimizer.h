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
#include "llvm/IR/LegacyPassManager.h"

#include "SimplexTuner.h"
#include "Optimizer.h"
#include "LoopTransformTree.h"
#include "Transformations.h"
#include "Debug.h"
#include "Tuners.h"

namespace clang {
namespace jit {

class TransformationTuner {
public:
  TransformationTuner(LoopTransformation& Transformation, unsigned MaxWithoutImprovement) : Transformation(Transformation), MaxWithoutImprovement(MaxWithoutImprovement) {
    HasTunableParams = Transformation.Space.getNumDimensions() > 0;
    if (HasTunableParams) {
      TTuner = createTuner(loadSearchAlgoEnv(), Transformation.Space);
    }
    JIT_INFO(dbgs() << "Tuning transformation of kind " << getTransformationName(Transformation.Kind) << "\n");
    NeedsUpdate = true;
    BestIdx = -1;
    RequestsSinceImprovement = 0;
  }

  ConfigEval getNext() {

    RequestsSinceImprovement++;
    NeedsUpdate = true;
    ConfigEval Request;
    if (HasTunableParams) {
      Request = TTuner->generateNextConfig();
    }
    Configs.push_back(Request);
    return Request;
  }

  llvm::Optional<ConfigEval> getBest() {
    if (NeedsUpdate)
      updateBest();
    if (BestIdx >= 0)
      return Configs[BestIdx];
    return None;
  }

  bool isDone() {
    if (!HasTunableParams && RequestsSinceImprovement > 0)
      return true;
    if (NeedsUpdate)
      updateBest();

    //outs() << "************************\n";
    //outs() << "No improvement for: " << RequestsSinceImprovement << "\n";
    return RequestsSinceImprovement >= MaxWithoutImprovement;
  }

  SearchSpace& getSearchSpace() {
    return Transformation.Space;
  }

  SmallVector<ConfigEval, 16> getAllConfigs() const {
    return Configs;
  }

private:
  void updateBest() {
    NeedsUpdate = false;
    int NewBestIdx = -1;
    TimingStats BestStats;

    for (auto I = 0; I < Configs.size(); I++) {
      auto& Cfg = Configs[I];
      if (Cfg.Stats->betterThan(BestStats)) {
        BestStats = *Cfg.Stats;
        NewBestIdx = I;
      }
    }
    if (NewBestIdx != BestIdx)
      RequestsSinceImprovement = 0;
    BestIdx = NewBestIdx;
  }
private:
  bool HasTunableParams;
  LoopTransformation& Transformation;
  std::unique_ptr<Tuner> TTuner;
  SmallVector<ConfigEval, 16> Configs;
  int BestIdx;
  unsigned RequestsSinceImprovement;
  unsigned MaxWithoutImprovement;
  bool NeedsUpdate;

};

struct DecisionNode {

  using NodePtr = std::unique_ptr<DecisionNode>;

  DecisionNode(TimingStats Baseline, LoopTransformation Trans, LoopTransformTreePtr Tree, DecisionNode* Parent = nullptr) : Baseline(Baseline), Transformation(Trans), LoopTree(std::move(Tree)), Parent(Parent) {
    assert(Baseline.Valid());

    TTuner = std::make_unique<TransformationTuner>(Transformation, getMaxEvalLimit());
    UnexploredIdx = 0;
  }

  unsigned getMaxEvalLimit() {
    switch(getDepth()) {
      case 1:
      case 2:
        return 35;
      case 3:
        return 20;
      case 4:
        return 15;
      default:
        return 10;
    }
    llvm_unreachable("");
  }

  static float getMultiplier(const LoopTransformation& LT) {
    switch(LT.Kind) {
      case LoopTransformation::TILE:
        return 1.5;
      case LoopTransformation::INTERCHANGE:
        return 1.3;
      case LoopTransformation::UNROLL_AND_JAM:
        return 1.1;
      case LoopTransformation::UNROLL:
        return 1.0;
      case LoopTransformation::ARRAY_PACK:
        return 1.0;
    }
    llvm_unreachable("");
  }

  float getUnexploredMultiplier() {
    if (isFullyExplored()) {
      // All children expanded, nothing to do
      return 0;
    }
    return getMultiplier(FeasibleTransformations[UnexploredIdx]);
  }

  bool isFullyExplored() {
    return UnexploredIdx >= FeasibleTransformations.size();
  }

  std::pair<float, DecisionNode*> getMostPromisingNode() {
    float ThisScore = computeScore();
    std::pair<float, DecisionNode*> MostPromising{ThisScore, this};
    for (auto& Child : Children) {
      if (!Child)
        continue;
      auto RecursiveBest = Child->getMostPromisingNode();
      if (RecursiveBest.first > MostPromising.first)
        MostPromising = RecursiveBest;
    }
    return MostPromising;
  }

  static constexpr float BadScore = -1;

  float computeScore() {
    float Multiplier = getUnexploredMultiplier();
    if (Multiplier == 0)
      return Multiplier;
    auto Best = TTuner->getBest();
    if (Best) {
      auto Stats = Best.getValue().Stats;
      if (!Stats->Valid())
        return BadScore;
      float Speedup = Baseline.Mean / Stats->Mean;
      return Speedup * Multiplier;
    }
    return BadScore;
  }

  DecisionNode& expand() {
    assert(UnexploredIdx < FeasibleTransformations.size() && "Node does not have any unexplored transformations");
    auto& Trans = FeasibleTransformations[UnexploredIdx];
    auto& Child = Children[UnexploredIdx];

    auto Best = TTuner->getBest();
    Child = std::make_unique<DecisionNode>(Baseline, Trans, TransformedTree->clone(), this);
//    Child->FullConfig = Best->Cfg;
//    Child->FullConfig.addAll(FullConfig);
    UnexploredIdx++;
    return *Child;
  }

  LoopTransformTreePtr getTransformedTree(ParamConfig& Cfg) {
    auto Tree = LoopTree->clone();
    apply(Transformation, *Tree, Cfg);
    return Tree;
  }

  LoopTransformTreePtr applyBestConfig() {
    auto Best = TTuner->getBest();
    if (!Best)
      return nullptr;
    return getTransformedTree(Best->Config);
  }

  bool isEvaluated() {
    return TTuner->isDone();
  }

  void finalize() {
    assert(isEvaluated() && "Node must be evaluated before new transformations can be found");
    assert(FeasibleTransformations.empty() && !TransformedTree && "Can only be called once"); // TODO: Allow to reevaluate with updated config
    TransformedTree = applyBestConfig();
    assert(TransformedTree);
    FeasibleTransformations = findTransformations(TransformedTree.get());
    std::sort(FeasibleTransformations.begin(), FeasibleTransformations.end(), [](const LoopTransformation& T1, const LoopTransformation& T2) {
      return getMultiplier(T1) > getMultiplier(T2);
    });
    Children.resize(FeasibleTransformations.size());
    UnexploredIdx = 0;
  }

  unsigned getDepth() const {
    if (!Parent) {
      return 1;
    }
    return Parent->getDepth()+1;
  }


  llvm::raw_ostream& printPath(llvm::raw_ostream& OS = outs()) {
    if (!Parent) {
      OS << "Root";
      return OS;
    }
    Parent->printPath(OS);
    OS << " --> " << getTransformationName(Transformation.Kind) << " [" << computeScore() << "]";
    return OS;
  }


  TimingStats Baseline;
  LoopTransformTreePtr LoopTree;
  LoopTransformTreePtr TransformedTree;
  std::unique_ptr<TransformationTuner> TTuner;
  LoopTransformation Transformation;
  DecisionNode* Parent;
//  KnobConfig FullConfig;
  SmallVector<LoopTransformation, 4> FeasibleTransformations;
  SmallVector<NodePtr, 4> Children;
  unsigned UnexploredIdx;
};

class TransformDecisionTree {
public:
  TransformDecisionTree(LoopTransformTreePtr OriginalTree, TimingStats BaselineStats)
    : Root(BaselineStats, LoopTransformation(), std::move(OriginalTree)) {
  }

  DecisionNode& getMostPromisingNode() {
    return *Root.getMostPromisingNode().second;
  }

  bool isFullyExplored() {
    return Root.isFullyExplored();
  }

  DecisionNode& getRoot() {
    return Root;
  }


private:
  DecisionNode Root;
};


class TransformTreeOptimizer: public Optimizer {
public:

  static std::once_flag IsPollyInitialized;

  TransformTreeOptimizer(clang::DiagnosticsEngine &Diags,
                         const clang::HeaderSearchOptions &HeaderOpts,
                         const clang::CodeGenOptions &CGOpts,
                         const clang::TargetOptions &TOpts, const clang::LangOptions &LOpts,
                         llvm::TargetMachine &TM)
      : Diags(Diags), HSOpts(HeaderOpts), CodeGenOpts(CGOpts),
        TargetOpts(TOpts), LangOpts(LOpts), TM(TM), ModToOptimize(nullptr), Done(false) {
  }

  void init(llvm::Module* M) override;

  ConfigEval optimize(llvm::Module *M, bool UseDefault) override;

//  const KnobSet& getKnobs() override {
//    return Knobs;
//  }

//  const SearchSpace& getSearchSpace() override {
//  }

  bool isDone() override {
    return Done;
  }

private:
  TargetIRAnalysis getTargetIRAnalysis();
  void createPasses(const llvm::Module &M, legacy::PassManager &PM,
                                            legacy::FunctionPassManager &FPM,
                                            ParamConfig &Cfg);

  float computeSpeedup(TimingStats& Stats);

private:
  using LoopTreeList = SmallVector<LoopTransformTreePtr, 2>;
  using LoopTreeIterator = LoopTreeList::iterator ;

  clang::DiagnosticsEngine &Diags;
  const clang::HeaderSearchOptions &HSOpts;
  const clang::CodeGenOptions &CodeGenOpts;
  const clang::TargetOptions &TargetOpts;
  const clang::LangOptions &LangOpts;
  llvm::TargetMachine &TM;
  Module *ModToOptimize;
  llvm::Optional<ConfigEval> BaseLine;

  // Trees corresponding to loop nests in the function
  LoopTreeList LoopTrees;

  // Iterator of the currently tuned loop nest
  LoopTreeIterator CurrentLoopTree;
  // Transformation decision tree for the current loop nest
  std::unique_ptr<TransformDecisionTree> DecisionTree;
  // Node that is currently investigated
  DecisionNode* CurrentNode;
  // Best node in the current tree, corresponds to a sequence of transformations
  std::pair<DecisionNode*, ConfigEval> BestNode;


  // Loop nests are tuned one after the other. This list contains the ones that have been finished.
  LoopTreeList FinalizedTrees;

 // KnobConfig FinalizedConfig;

  // All loop nests have been tuned.
  bool Done;

};


}
}


#endif //LLVM_TRANSFORMTREESEARCH_H
