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

class LoopTransformTreeApplicator : public llvm::LoopPass {

private:
  ArrayRef<LoopTransformTree> LoopTrees;


  bool matchTree(LoopTransformTree& Tree, Loop* L) {
    auto Name = getLoopName(L);
    if (Name.empty())
      return false;
    auto Node = Tree.getNode(Name);
    if (!Node)
      return false;
    assert(!Node->isVirtual() && "Actual loops cannot be virtual - there is a bug in the tree creation pass");
    assert(Node->getOriginalLoop() == Node && "Matched loop is not the original one!");

    do {
      auto& Attributes = Node->getAttributes();
      // TODO: Write metadata to IR
    } while();
  }

public:
  static char ID;

  explicit LoopTransformTreeApplicator(ArrayRef<LoopTransformTree> LoopTrees) : LoopPass(ID), LoopTrees(LoopTrees){};

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {

    if (Loop->getHeader()->getParent()->isDeclarationForLinker()) {
      // We don't want to consider loops in functions that are marked available_externally
      return false;
    }

    for (auto& Tree : LoopTrees) {
      if (matchTree(Tree), Loop)
        return true;
    }

    return false;

  }

}; // end class

char LoopTransformTreeApplicator::ID = 0;
static RegisterPass<LoopTransformTreeApplicator> Register("loop-tree-applicator",
                                              "Make loop transformation tree",
                                              false /* only looks at CFG*/,
                                              false /* analysis pass */);

llvm::Pass *createLoopTransformTreeApplicatorPass(SmallVectorImpl<LoopTransformTree> &LoopTrees) {
  auto LTC = new LoopTransformTreeApplicator(LoopTrees);
  return LTC;
}

}
}