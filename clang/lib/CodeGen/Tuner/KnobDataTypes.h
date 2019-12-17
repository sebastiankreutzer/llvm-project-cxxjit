//
// Created by sebastian on 30.09.19.
//

#ifndef CLANG_KNOBDATATYPES_H
#define CLANG_KNOBDATATYPES_H

namespace llvm {
struct raw_ostream;
}

namespace clang {
namespace jit {

class LoopKnob;

// NOTE: Using "Name" here instead of "ID" to differentiate from Loop::getID()
// which returns the loop MDNode.
using LoopName = unsigned;

struct LoopTransformConfig {

  // DisableLoopTransform and DisableNonForced are not tuned, thus not counted
  enum Parameter {
    INTERLEAVE_COUNT,
    VECTORIZE_PREDICATE_ENABLE,
    VECTORIZE_WIDTH,
    DISABLE_LICM,
    DISABLE_LICM_VERSIONING,
    DISTRIBUTE,
    UNROLL_COUNT,
    UNROLL_AND_JAM,
    NUM_PARAMS
  };

  //  static constexpr unsigned INTERLEAVE_COUNT_MIN = 0;
  //  static constexpr unsigned INTERLEAVE_COUNT_MAX = 5;
  //  static constexpr unsigned VECTORIZE_WIDTH_MIN = 0;
  //  static constexpr unsigned VECTORIZE_WIDTH_MAX = 5;
  //  static constexpr unsigned UNROLL_COUNT_MIN = 0;
  //  static constexpr unsigned UNROLL_COUNT_MAX = 6;

  // These are the default min and max values
  static constexpr unsigned MIN_VALS[NUM_PARAMS] = {0, 0, 0, 0, 0, 0, 0, 0};
  static constexpr unsigned MAX_VALS[NUM_PARAMS] = {3, 1, 5, 1, 1, 1, 3, 1};

  unsigned Vals[NUM_PARAMS] = {3, 1, 3, 0, 0, 1, 3, 1};

  bool DisableLoopTransform{false};

  // For members with the suffix "Exp", the value is logarithmic
  //  unsigned InterleaveCountExp{3};
  //  bool VectorizePredicateEnable{true};
  //  unsigned VectorizeWidthExp{3};
  //  bool DisableLICM{false};
  //  bool DisableLICMVersioning{false};
  //  bool Distribute{true};
  //
  //  unsigned UnrollCountExp{4};

  // NOTE: Instead of setting llvm.unroll.full it's almost certainly better to
  // just work with high unroll counts.
  //       This prevents big jumps when generating neighboring configs.
  // bool UnrollFull{false};

  //  bool UnrollAndJam{true};

  bool DisableNonForced{true};

  bool getVectorizePredicateEnabled() const {
    return (bool) Vals[VECTORIZE_PREDICATE_ENABLE];
  }

  bool getDisableLICM() const { return (bool) Vals[DISABLE_LICM]; }

  bool getDisableLICMVersioning() const {
    return (bool) Vals[DISABLE_LICM_VERSIONING];
  }

  bool getDistribute() const { return (bool) Vals[DISTRIBUTE]; }

  bool getUnrollAndJam() const { return (bool) Vals[UNROLL_AND_JAM]; }

  unsigned getInterleaveCount() const {
    return (unsigned) 1 << Vals[INTERLEAVE_COUNT];
  }

  unsigned getVectorizeWidth() const {
    return (unsigned) 1 << Vals[VECTORIZE_WIDTH];
  }

  unsigned getUnrollCount() const { return (unsigned) 1 << Vals[UNROLL_COUNT]; }

  void dump(llvm::raw_ostream &OS, unsigned Indent = 0) const;

  void dump(llvm::raw_ostream &OS, const LoopKnob &Knob, unsigned Indent = 0) const;

};

llvm::raw_ostream &operator<<(llvm::raw_ostream &OS,
                              const LoopTransformConfig &);

}
}
#endif // CLANG_KNOBDATATYPES_H
