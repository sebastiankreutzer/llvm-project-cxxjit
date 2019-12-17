//
// Created by sebastian on 27.09.19.
//

#include "LoopTransformTree.h"
#include "Debug.h"
#include "LoopMD.h"
#include "llvm/Analysis/LoopPass.h"

using namespace llvm;

namespace clang {
namespace jit {

class LoopTransformTreeCreator : public llvm::LoopPass {

private:
  SmallVectorImpl<LoopTransformTree>& LoopTrees;


  LoopNode* createSubTree(LoopTransformTree& Tree, Loop* Loop, LoopNode* Parent) {

    auto LoopMD = getOrCreateLoopID(Loop);
    LoopNode* Root = Tree.makeNode();
    auto LoopName = assignLoopName(Loop, Root->getLoopName());
    for (auto L : Loop->getSubLoops()) {
      auto SubTree = createSubTree(Tree, L, Root);
      Root->addSubLoop(SubTree);
    }

    return Root;
  }

public:
  static char ID;

  explicit LoopTransformTreeCreator(SmallVectorImpl<LoopTransformTree>& LoopTrees) : LoopPass(ID), LoopTrees(LoopTrees){};

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {

    if (Loop->getHeader()->getParent()->isDeclarationForLinker()) {
      // We don't want to consider loops in functions that are marked available_externally
      return false;
    }

    if (!Loop->getParentLoop()) {
      LoopTransformTree Tree;
      auto Root = createSubTree(Tree, Loop, nullptr);
      Tree.setRoot(Root);
      LoopTrees.push_back(std::move(Tree));
      return true;
    }
    return false;

  }

}; // end class

char LoopTransformTreeCreator::ID = 0;
static RegisterPass<LoopTransformTreeCreator> Register("loop-tree-creator",
                                              "Make loop transformation tree",
                                              false /* only looks at CFG*/,
                                              false /* analysis pass */);

llvm::Pass *createLoopTransformTreeCreatorPass(SmallVectorImpl<LoopTransformTree> &LoopTrees) {
  auto LTC = new LoopTransformTreeCreator(LoopTrees);
  return LTC;
}

}
}