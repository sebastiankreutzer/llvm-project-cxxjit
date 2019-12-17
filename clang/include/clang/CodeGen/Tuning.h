//
// Created by sebastian on 30.10.19.
//

#ifndef CLANG_JIT_H
#define CLANG_JIT_H

#include <tuple>
#include <functional>

namespace clang {
namespace jit {

template<typename T>
struct tunable_range {
  tunable_range(T Min, T Max) : Min(Min), Max(Max) {};

  T Min;
  T Max;
};

}

}

#endif //CLANG_JIT_H
