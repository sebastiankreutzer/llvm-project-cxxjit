//
// Created by sebastian on 20.12.19.
//

#ifndef LLVM_TRANSFORMATIONS_H
#define LLVM_TRANSFORMATIONS_H

#include "KnobSet.h"
#include "LoopTransformTree.h"

namespace clang {
namespace jit {

class LoopTransformTree;

struct LoopTransform {
  enum TransformKind {
    TILE, INTERCHANGE
  };

  TransformKind Kind;
  KnobSet Knobs;

};

void findTransformations(LoopNode* Root, SmallVectorImpl<LoopTransform>& Transformations);

SmallVector<LoopTransform, 4> findTransformations(LoopTransformTree* Tree);


}
}

#endif //LLVM_TRANSFORMATIONS_H
