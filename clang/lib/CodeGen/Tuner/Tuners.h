//
// Created by sebastian on 06.12.19.
//

#ifndef CLANG_TUNERS_H
#define CLANG_TUNERS_H

#include "Tuner.h"
#include "SimplexTuner.h"
#include "ModifiedSimplexTuner.h"


namespace clang {
namespace jit {

enum class TunerSearchAlgo {
  Random, Simplex, ModSimplex
};

inline TunerSearchAlgo loadSearchAlgoEnv() {
  auto DriverStr = std::getenv("CJ_SEARCH");
  if (DriverStr) {
    if (std::strcmp(DriverStr, "simplex") == 0)
      return TunerSearchAlgo::Simplex;
    if (std::strcmp(DriverStr, "modsimplex") == 0)
      return TunerSearchAlgo::ModSimplex;
  }

  return TunerSearchAlgo::Random;

}

inline std::unique_ptr<Tuner> createTuner(TunerSearchAlgo Search, SearchSpace& Space) {
  switch(Search) {
    case TunerSearchAlgo::Simplex:
      return std::make_unique<SimplexTuner>(Space);
    case TunerSearchAlgo::ModSimplex:
      return std::make_unique<ModifiedSimplexTuner>(Space);
    case TunerSearchAlgo::Random:
    default:
      return std::make_unique<RandomTuner>(Space);
  }
}

}
}

#endif //CLANG_TUNERS_H
