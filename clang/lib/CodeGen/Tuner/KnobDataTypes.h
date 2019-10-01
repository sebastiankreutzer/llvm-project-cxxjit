//
// Created by sebastian on 30.09.19.
//

#ifndef CLANG_KNOBDATATYPES_H
#define CLANG_KNOBDATATYPES_H

namespace tuner {

// NOTE: Using "Name" here instead of "ID" to differentiate from Loop::getID() which returns the loop MDNode.
using LoopName = unsigned;

struct LoopTransformConfig {
  bool DisableNonForced{false};

};


}

#endif //CLANG_KNOBDATATYPES_H
