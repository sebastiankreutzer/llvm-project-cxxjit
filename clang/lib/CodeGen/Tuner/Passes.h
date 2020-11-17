//
// Created by sebastian on 27.09.19.
//

#ifndef CLANG_PASSES_H
#define CLANG_PASSES_H

#include "LoopTransformTree.h"

namespace llvm {
class Pass;
}

namespace clang {
namespace jit {

Pass *createLoopTransformTreeCreatorPass(SmallVectorImpl<LoopTransformTreePtr> &LoopTrees);

Pass *createLoopTransformTreeApplicatorPass(ArrayRef<LoopTransformTree*> LoopTrees);

}
}

#endif // CLANG_PASSES_H
