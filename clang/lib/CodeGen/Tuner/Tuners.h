//
// Created by sebastian on 06.12.19.
//

#ifndef CLANG_TUNERS_H
#define CLANG_TUNERS_H

#include "Tuner.h"
#include "SimplexTuner.h"

using namespace tuner;

namespace clang {
namespace jit {

enum class TunerSearchAlgo {
  Random, Simplex
};

inline TunerSearchAlgo loadSearchAlgoEnv() {
  auto DriverStr = std::getenv("CJ_SEARCH");
  if (DriverStr && std::strcmp(DriverStr, "simplex") == 0) {
    return TunerSearchAlgo::Simplex;
  } else {
    return TunerSearchAlgo::Random;
  }
}

inline std::unique_ptr<Tuner> createTuner(TunerSearchAlgo Search, KnobSet& Knobs) {
  switch(Search) {
    case TunerSearchAlgo::Simplex:
      return llvm::make_unique<SimplexTuner>(Knobs);
    case TunerSearchAlgo::Random:
    default:
      return llvm::make_unique<RandomTuner>(Knobs);
  }
}

}
}

#endif //CLANG_TUNERS_H
