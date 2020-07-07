//
// Created by sebastian on 27.09.19.
//

#include <llvm/Transforms/Scalar/IndVarSimplify.h>
#include "LoopTransformTree.h"
#include "Debug.h"
#include "LoopMD.h"
#include "llvm/Analysis/LoopPass.h"
#include "llvm/Transforms/Utils/LoopUtils.h"

using namespace llvm;

namespace clang {
namespace jit {

class LoopTransformTreeCreator : public llvm::LoopPass {

private:
  SmallVectorImpl<LoopTransformTreePtr>* LoopTrees;

  LoopNode* createSubTree(LoopTransformTree& Tree, Loop* L, ScalarEvolution& SE, LoopNode* Parent) {

    // NOTE: Trip count estimation based on LoopUnrollPass

    // Find trip count and trip multiple if count is not available
    unsigned TripCount = 0;
    unsigned TripMultiple = 1;
    // If there are multiple exiting blocks but one of them is the latch, use the
    // latch for the trip count estimation. Otherwise insist on a single exiting
    // block for the trip count estimation.
    BasicBlock *ExitingBlock = L->getLoopLatch();
    if (!ExitingBlock || !L->isLoopExiting(ExitingBlock))
      ExitingBlock = L->getExitingBlock();
    if (ExitingBlock) {
      TripCount = SE.getSmallConstantTripCount(L, ExitingBlock);
      TripMultiple = SE.getSmallConstantTripMultiple(L, ExitingBlock);
    }

    // Try to find the trip count upper bound if we cannot find the exact trip
    // count.
    unsigned MaxTripCount = 0;
    bool MaxOrZero = false;
    if (!TripCount) {
      MaxTripCount = SE.getSmallConstantMaxTripCount(L);
      MaxOrZero = SE.isBackedgeTakenCountMaxOrZero(L);
    }

    auto LoopMD = getOrCreateLoopID(L);
    LoopNode* Root = Tree.makeNode();

    Root->getTripCountInfo() = LoopNode::TripCountInfo(TripCount > 0 ? TripCount : MaxTripCount, TripCount > 0);
    Root->addTagAttribute(MDTags::DISABLE_NONFORCED, true);
    auto LoopName = assignLoopName(L, Root->getLoopName());
    JIT_INFO(outs() << "Trip count for loop " << Root->getLoopName() << " is " << Root->getTripCountInfo().TripCount << ", exact: " << Root->getTripCountInfo().IsExact << "\n");
    for (auto L : L->getSubLoops()) {
      auto SubTree = createSubTree(Tree, L, SE, Root);
      Root->addSubLoop(SubTree);
    }

    return Root;
  }

public:
  static char ID;

  explicit LoopTransformTreeCreator() : LoopPass(ID){};

  void writeResultsTo(SmallVectorImpl<LoopTransformTreePtr>& LoopTrees) {
    this->LoopTrees = &LoopTrees;
  }

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {
    assert(LoopTrees && "Result vector not set");

    if (Loop->getHeader()->getParent()->isDeclarationForLinker()) {
      // We don't want to consider loops in functions that are marked available_externally
      return false;
    }

    ScalarEvolution &SE = getAnalysis<ScalarEvolutionWrapperPass>().getSE();

    if (!Loop->getParentLoop()) {
      LoopTransformTreePtr Tree = std::make_unique<LoopTransformTree>();
      auto Root = createSubTree(*Tree, Loop, SE, nullptr);
      Tree->setRoot(Root);
      LoopTrees->push_back(std::move(Tree));
      return true;
    }
    return false;

  }

  void getAnalysisUsage(AnalysisUsage& AU) const override {
    AU.addRequired<ScalarEvolutionWrapperPass>();
    getLoopAnalysisUsage(AU);
  }

}; // end class

char LoopTransformTreeCreator::ID = 0;
static RegisterPass<LoopTransformTreeCreator> Register("loop-tree-creator",
                                              "Make loop transformation tree",
                                              false /* only looks at CFG*/,
                                              false /* analysis pass */);

llvm::Pass *createLoopTransformTreeCreatorPass(SmallVectorImpl<LoopTransformTreePtr> &LoopTrees) {
  auto LTC = new LoopTransformTreeCreator();
  LTC->writeResultsTo(LoopTrees);
  return LTC;
}

}
}