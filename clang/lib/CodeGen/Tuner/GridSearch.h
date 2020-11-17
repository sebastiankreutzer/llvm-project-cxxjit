//
// Created by sebastian on 03.11.20.
//

#ifndef LLVM_GRIDSEARCH_H
#define LLVM_GRIDSEARCH_H

#include "Tuner.h"

namespace clang {
namespace jit {


class GridSearch : public Tuner {
public:
  explicit GridSearch(SearchSpace& Space)
  : Space(Space), NextConfig(Space) {
    for (unsigned I = 0; I < Space.getNumDimensions(); I++) {
      assert(Space.getDim(I).Type == ParamType::INT && "Floating point parameters not supported yet");
      NextConfig[I] = Space.getDim(I).Min;
    }
  }

  ConfigEval generateNextConfig() override;

  bool isDone() override {
    return Done;
  }

private:
  SearchSpace& Space;
  ParamConfig NextConfig;

  bool Done{false};
};

}
}

#endif //LLVM_GRIDSEARCH_H
