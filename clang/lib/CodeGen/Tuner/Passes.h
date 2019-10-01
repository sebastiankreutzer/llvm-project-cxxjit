//
// Created by sebastian on 27.09.19.
//

#ifndef CLANG_PASSES_H
#define CLANG_PASSES_H

namespace llvm {
  class Pass;
}

namespace tuner {

  Pass* createLoopKnobCreatorPass(KnobSet& KS);

}

#endif //CLANG_PASSES_H
