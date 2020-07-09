//
// Created by sebastian on 20.12.19.
//

#ifndef LLVM_TRANSFORMATIONS_H
#define LLVM_TRANSFORMATIONS_H

#include "LoopTransformTree.h"
#include "Search.h"

namespace clang {
namespace jit {

class LoopTransformTree;

namespace transform_defaults {
constexpr unsigned UNROLL_MIN = 1;
constexpr unsigned UNROLL_MAX = 64;
constexpr unsigned UNROLL_AND_JAM_MIN = 1;
constexpr unsigned UNROLL_AND_JAM_MAX = 16;

constexpr unsigned UNROLL_DFLT = 8;
constexpr unsigned TILE_MIN = 1;
constexpr unsigned TILE_MAX = 4096; // TODO: Use cache size as baseline
constexpr unsigned TILE_DFLT = 512; // TODO: Use cache size as baseline

// Loops with smaller trip counts should not be tiled.
constexpr unsigned MIN_TILING_TRIP_COUNT = 8;
}

struct LoopTransformation {
  enum TransformKind {
    NONE,
    TILE,
    INTERCHANGE,
    UNROLL,
    UNROLL_AND_JAM,
    ARRAY_PACK,
    VECTORIZE
  };

  TransformKind Kind{NONE};
  SmallString<8> Root;
  // Used to store information such as the interchange permutations.
  SmallVector<int, 4> IntParams;
  SearchSpace Space;
  // Used for tunable parameters that have only one legal value.
  // This avoids having to handle this case in the search heuristic itself.
  // Example: Tiling a loop with known trip count of 1.
  SmallVector<ParamVal, 2> FixedParams;
//  KnobSet Knobs;
  // NOTE: Identifying knobs with strings is probably not very efficient but avoids the need for polymorphism.
//  StringMap<SmallVector<KnobID, 4>> KnobMap;

  void addSearchDim(SearchDim Dim) {
    Space.addDim(std::move(Dim));
  }

//  void addKnob(IntKnob* Knob, StringRef Category = "") {
//    Knobs.add(Knob);
//    if (!Category.empty()) {
//      KnobMap[Category].push_back(Knob->getID());
//    }
//  }
//
//  ArrayRef<KnobID> getKnobs(StringRef Category) const {
//    auto It = KnobMap.find(Category);
//    if (It == KnobMap.end()) {
//      return {};
//    }
//    return It->second;
//  }

};

inline const char* getTransformationName(LoopTransformation::TransformKind Kind) {
  switch(Kind) {
    case LoopTransformation::TILE:
      return "TILE";
    case LoopTransformation::INTERCHANGE:
      return "INTERCHANGE";
    case LoopTransformation::UNROLL:
      return "UNROLL";
    case LoopTransformation::UNROLL_AND_JAM:
      return "UNROLL_AND_JAM";
    case LoopTransformation::ARRAY_PACK:
      return "ARRAY_PACK";
    case LoopTransformation::VECTORIZE:
      return "VECTORIZE";
  }
  return "UNKNOWN";
}

void findTransformations(LoopNode* Root, SmallVectorImpl<LoopTransformation>& Transformations);

SmallVector<LoopTransformation, 4> findTransformations(LoopTransformTree* Tree);

void apply(LoopTransformation& Transformation, LoopTransformTree& Tree, ParamConfig& Cfg);

void applyUnrollAndJam(LoopNode* Root, ArrayRef<unsigned> Counts);

void applyUnroll(LoopNode* Root, ArrayRef<unsigned> Counts);

void applyTiling(LoopNode *Root, unsigned Depth, ArrayRef<unsigned> Sizes, StringRef PeelType = "rectangular");

void applyInterchange(LoopNode *Root, unsigned Depth, ArrayRef<int> Permutation);

void viewTree(LoopTransformTree& Tree);


}
}

#endif //LLVM_TRANSFORMATIONS_H
