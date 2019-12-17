//
// Created by sebastian on 27.09.19.
//

#ifndef CLANG_LOOPKNOB_H
#define CLANG_LOOPKNOB_H

#include <llvm/ADT/SmallString.h>
#include "Knob.h"
#include "KnobSet.h"

namespace llvm {
class Pass;
}

namespace clang {
namespace jit {

class LoopKnob : public Knob<LoopTransformConfig> {
  // LoopName Name;

public:
  explicit LoopKnob(std::string IndVarName = "unknown") :
      InductionVarName(std::move(IndVarName)) {
    // Set default min/max values
    for (auto i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
      MinVals[i] = LoopTransformConfig::MIN_VALS[i];
      MaxVals[i] = LoopTransformConfig::MAX_VALS[i];
      Defaults[i] = MinVals[i];
    }
  }

  void setAttributeBounds(LoopTransformConfig::Parameter Attr, unsigned Min, unsigned Max, unsigned Default) {
    MinVals[Attr] = Min;
    MaxVals[Attr] = Max;
    Defaults[Attr] = Default;
  }

  void disableTuning(LoopTransformConfig::Parameter Attr, unsigned Val) {
    MinVals[Attr] = Val;
    MaxVals[Attr] = Val;
    Defaults[Attr] = Val;
  }

  bool isTunable(LoopTransformConfig::Parameter Attr) const {
    return MinVals[Attr] < MaxVals[Attr];
  }

  unsigned getMin(LoopTransformConfig::Parameter Attr) const {
    return MinVals[Attr];
  }

  unsigned getMax(LoopTransformConfig::Parameter Attr) const {
    return MaxVals[Attr];
  }

  unsigned getDefault(LoopTransformConfig::Parameter Attr) const {
    return Defaults[Attr];
  }

  unsigned getTunableDimension() const {
    unsigned Dim = 0;
    for (int i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
      if (isTunable((LoopTransformConfig::Parameter) i)) {
        Dim++;
      }
    }
    return Dim;
  }

  LoopTransformConfig getDefault() const override {
    LoopTransformConfig LTC;
    for (int i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
      LTC.Vals[i] = Defaults[i];
    }
    return LTC;
  }

  LoopTransformConfig getVal(const KnobConfig &Cfg) const override {
    auto It = Cfg.LoopCfg.find(getID());
    assert(It != Cfg.LoopCfg.end() &&
           "Config is expected to be initialized here");
    return It->second;
  }

  void setVal(KnobConfig &Cfg, LoopTransformConfig LTC) override {
    Cfg.LoopCfg[getID()] = LTC;
  }

  void addSubLoopKnob(LoopKnob *LK) {
    SubLoopKnobs.push_back(LK);
  }

  llvm::iterator_range<SmallVector<LoopKnob *, 2>::iterator> subLoopKnobs() {
    return llvm::make_range(SubLoopKnobs.begin(), SubLoopKnobs.end());
  }

  std::string getName() const override {
    return "Loop knob (id=" + std::to_string(getID()) + ", name=" + InductionVarName + ", " +
           std::to_string(SubLoopKnobs.size()) + " subloops)";
  }

private:
  unsigned MinVals[LoopTransformConfig::NUM_PARAMS];
  unsigned MaxVals[LoopTransformConfig::NUM_PARAMS];
  unsigned Defaults[LoopTransformConfig::NUM_PARAMS];
  std::string InductionVarName;
  SmallVector<LoopKnob *, 2> SubLoopKnobs;
};

template<typename RNETy>
LoopTransformConfig createRandomLoopConfig(LoopKnob &K, RNETy &RNE) {

  auto BiasedFlip = [&RNE](unsigned TrueBias) -> bool {
    std::uniform_int_distribution<unsigned> Dist(0, 99);
    return Dist(RNE) < TrueBias;
  };

  auto BiasedFlipOrDefault = [&RNE, &K, &BiasedFlip](LoopTransformConfig::Parameter Attr,
                                                     unsigned TrueBias) -> unsigned {
    return K.isTunable(Attr) ? (unsigned) BiasedFlip(TrueBias) : K.getDefault(Attr);
  };

  // TODO: Review biases for boolean values
  LoopTransformConfig Cfg;

  // Enables attributes actually being applied
  Cfg.DisableLoopTransform = false;

  std::uniform_int_distribution<unsigned> VecWidthDist(
      K.getMin(LoopTransformConfig::VECTORIZE_WIDTH),
      K.getMax(LoopTransformConfig::VECTORIZE_WIDTH));
  Cfg.Vals[LoopTransformConfig::VECTORIZE_WIDTH] = VecWidthDist(RNE);

  std::uniform_int_distribution<unsigned> InterleaveCountDist(
      K.getMin(LoopTransformConfig::INTERLEAVE_COUNT),
      K.getMax(LoopTransformConfig::INTERLEAVE_COUNT));
  Cfg.Vals[LoopTransformConfig::INTERLEAVE_COUNT] = InterleaveCountDist(RNE);

  // Probably no reason why this should be disabled...
  Cfg.Vals[LoopTransformConfig::VECTORIZE_PREDICATE_ENABLE] = BiasedFlipOrDefault(
      LoopTransformConfig::VECTORIZE_PREDICATE_ENABLE, 90);

  // Probably always beneficial?
  Cfg.Vals[LoopTransformConfig::DISABLE_LICM] = BiasedFlipOrDefault(LoopTransformConfig::DISABLE_LICM, 10);

  Cfg.Vals[LoopTransformConfig::DISABLE_LICM_VERSIONING] = BiasedFlipOrDefault(
      LoopTransformConfig::DISABLE_LICM_VERSIONING, 30);

  Cfg.Vals[LoopTransformConfig::DISTRIBUTE] = BiasedFlipOrDefault(LoopTransformConfig::DISTRIBUTE, 50);

  std::uniform_int_distribution<unsigned> UnrollDist(
      K.getMin(LoopTransformConfig::UNROLL_COUNT),
      K.getMax(LoopTransformConfig::UNROLL_COUNT));
  Cfg.Vals[LoopTransformConfig::UNROLL_COUNT] = UnrollDist(RNE);

  Cfg.Vals[LoopTransformConfig::UNROLL_AND_JAM] = BiasedFlipOrDefault(LoopTransformConfig::UNROLL_AND_JAM, 50);

  Cfg.DisableNonForced = true;

  return Cfg;
}

}
}

#endif // CLANG_LOOPKNOB_H
