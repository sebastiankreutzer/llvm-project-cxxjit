//
// Created by sebastian on 30.09.19.
//

#ifndef CLANG_KNOBDATATYPES_H
#define CLANG_KNOBDATATYPES_H

namespace llvm {
  struct raw_ostream;
}

namespace tuner {

// NOTE: Using "Name" here instead of "ID" to differentiate from Loop::getID() which returns the loop MDNode.
using LoopName = unsigned;

struct LoopTransformConfig {

  static constexpr unsigned INTERLEAVE_COUNT_MIN = 0;
  static constexpr unsigned INTERLEAVE_COUNT_MAX = 5;
  static constexpr unsigned VECTORIZE_WIDTH_MIN = 0;
  static constexpr unsigned VECTORIZE_WIDTH_MAX = 5;
  static constexpr unsigned UNROLL_COUNT_MIN = 0;
  static constexpr unsigned UNROLL_COUNT_MAX = 6;

  bool DisableLoopTransform{true};

  // For members with the suffix "Exp", the value is logarithmic
  unsigned InterleaveCountExp{3};
  bool VectorizePredicateEnable{true};
  unsigned VectorizeWidthExp{3};
  bool DisableLICM{false};
  bool DisableLICMVersioning{false};
  bool Distribute{true};

  unsigned UnrollCountExp{4};

  // NOTE: Instead of setting llvm.unroll.full it's almost certainly better to just work with high unroll counts.
  //       This prevents big jumps when generating neighboring configs.
  // bool UnrollFull{false};

  bool UnrollAndJam{true};

  bool DisableNonForced{true};

  unsigned getInterleaveCount() const {
    return 1 << InterleaveCountExp;
  }

  unsigned getVectorizeWidth() const {
    return 1 << VectorizeWidthExp;
  }

  unsigned getUnrollCount() const {
    return 1 << UnrollCountExp;
  }

  void dump(llvm::raw_ostream& OS, unsigned Indent = 0) const;
};

llvm::raw_ostream& operator<<(llvm::raw_ostream& OS, const LoopTransformConfig&);


}

#endif //CLANG_KNOBDATATYPES_H
