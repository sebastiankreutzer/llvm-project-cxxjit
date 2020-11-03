//
// Created by sebastian on 03.11.20.
//

#include "GridSearch.h"

namespace clang {
namespace jit{

ConfigEval GridSearch::generateNextConfig() {
  auto Eval = ConfigEval(NextConfig, "GRID");
  for (unsigned Dim = 0; Dim < Space.getNumDimensions(); Dim++) {
    if (NextConfig[Dim] < Space[Dim].Max) {
      NextConfig[Dim] += 1;
      break;
    } else {
      NextConfig[Dim] = Space[Dim].Min;
      Done = Dim == Space.getNumDimensions()-1;
    }
  }
//  llvm::outs() << "Grid: ";
//  for (auto Val : Eval.Config.Values) {
//    if (auto V = Val.getIntVal()) {
//      llvm::outs() << *V << ", ";
//    }
//  }
//  llvm::outs() << "\n";
  return Eval;
}


}
}
