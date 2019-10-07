//
// Created by sebastian on 24.09.19.
//

#include "SimpleKnobs.h"

#ifndef CLANG_CODEGENKNOBS_H
#define CLANG_CODEGENKNOBS_H

namespace tuner {

class OptLvlKnob : public IntKnob {
public:
  OptLvlKnob() : IntKnob(0, 3, 3, "OptLvl") {};
};

class OptSizeKnob : public IntKnob {
public:
  OptSizeKnob() : IntKnob(0, 2, 0, "OptSizeLvl") {};
};

}

#endif //CLANG_CODEGENKNOBS_H
