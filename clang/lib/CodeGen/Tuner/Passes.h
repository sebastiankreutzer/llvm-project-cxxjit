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


Pass *createLoopKnobCreatorPass(KnobSet &KS);

Pass *createLoopTransformTreeCreatorPass(SmallVectorImpl<LoopTransformTree> &LoopTrees);

Pass *createApplyLoopKnobPass(KnobConfig &KnobCfg);

}
}

#endif // CLANG_PASSES_H
