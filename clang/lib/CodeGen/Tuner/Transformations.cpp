//
// Created by sebastian on 20.12.19.
//

#include "Transformations.h"
#include "SimpleKnobs.h"

namespace clang {
namespace jit {

void findTilingTransformations(LoopNode *Root, SmallVectorImpl<LoopTransform> &Transformations) {
  if (!Root->isTightlyNested()) {
    for (auto& SubLoop : Root->subLoops()) {
      findTilingTransformations(&*SubLoop, Transformations);
    }
    return;
  }
  unsigned Depth = Root->getRelativeMaxDepth();
  LoopTransform Trans;
  LoopNode* Node = Root;
  KnobSet Knobs;
  // TODO: Does not respect successor relationship
  for (int i = 1; i <= Depth; i++) {
    auto Name = "Loop " + Node->getLoopName() + " - Tile Size";
    IntKnob* TilingSize = new IntKnob(1, 1024, 1, Name.str()); // TODO: Heuristic for max value (also memory leak)
    Knobs.add(TilingSize);
    Node = Node->getFirstSubLoop();
  }
  Trans.Knobs = std::move(Knobs);
  Trans.Kind = LoopTransform::TILE;
  Transformations.push_back(Trans);
}

void findTransformations(LoopNode* Root, SmallVectorImpl<LoopTransform>& Transformations) {
  findTilingTransformations(Root, Transformations);
}

SmallVector<LoopTransform, 4> findTransformations(LoopTransformTree *Tree) {
  assert(Tree && "Tree is null");
  SmallVector<LoopTransform, 4> Transformations;
  auto Root = Tree->getRoot();
  if (Root) {
    findTransformations(Root, Transformations);
  }
  return Transformations;
}

}
}