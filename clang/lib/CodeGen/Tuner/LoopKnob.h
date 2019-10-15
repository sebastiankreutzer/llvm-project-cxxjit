//
// Created by sebastian on 27.09.19.
//

#ifndef CLANG_LOOPKNOB_H
#define CLANG_LOOPKNOB_H

#include "Knob.h"
#include "KnobSet.h"

namespace llvm {
class Pass;
}

namespace tuner {

template<typename RNETy>
LoopTransformConfig createRandomLoopConfig(RNETy& RNE) {

  auto BiasedFlip = [&RNE](unsigned TrueBias) -> bool {
    std::uniform_int_distribution<unsigned> Dist(0, 99);
    return Dist(RNE) < TrueBias;
  };

  // TODO: Review biases for boolean values
  LoopTransformConfig Cfg;

  // Enables attributes actually being applied
  Cfg.DisableLoopTransform = false;


  std::uniform_int_distribution<unsigned> VecWidthDist(LoopTransformConfig::MIN_VALS[LoopTransformConfig::VECTORIZE_WIDTH],
      LoopTransformConfig::MAX_VALS[LoopTransformConfig::VECTORIZE_WIDTH]);
  Cfg.Vals[LoopTransformConfig::VECTORIZE_WIDTH] = VecWidthDist(RNE);

  // Interleave count should not be greater than the vectorization width
  const unsigned InterleaveMax = LoopTransformConfig::MAX_VALS[LoopTransformConfig::INTERLEAVE_COUNT];
  std::uniform_int_distribution<unsigned> InterleaveCountDist(LoopTransformConfig::MIN_VALS[LoopTransformConfig::INTERLEAVE_COUNT],
      std::min(Cfg.Vals[LoopTransformConfig::VECTORIZE_WIDTH], InterleaveMax));
  Cfg.Vals[LoopTransformConfig::INTERLEAVE_COUNT] = InterleaveCountDist(RNE);

  // Probably no reason why this should be disabled...
  Cfg.Vals[LoopTransformConfig::VECTORIZE_PREDICATE_ENABLE] = (unsigned) BiasedFlip(90);

  // Probably always beneficial?
  Cfg.Vals[LoopTransformConfig::DISABLE_LICM] = (unsigned) BiasedFlip(10);

  Cfg.Vals[LoopTransformConfig::DISABLE_LICM_VERSIONING] = (unsigned) BiasedFlip(30);

  Cfg.Vals[LoopTransformConfig::DISTRIBUTE] = (unsigned) BiasedFlip(50);

  std::uniform_int_distribution<unsigned> UnrollDist(LoopTransformConfig::MIN_VALS[LoopTransformConfig::UNROLL_COUNT],
      LoopTransformConfig::MAX_VALS[LoopTransformConfig::UNROLL_COUNT]);
  Cfg.Vals[LoopTransformConfig::UNROLL_COUNT] = UnrollDist(RNE);

  Cfg.Vals[LoopTransformConfig::UNROLL_AND_JAM] = (unsigned) BiasedFlip(50);

  Cfg.DisableNonForced = true;

  return Cfg;
}


class LoopKnob : public Knob<LoopTransformConfig> {
  //LoopName Name;

public:
  explicit LoopKnob(/*LoopName Name*/) /*:
    Name(Name)*/
  {}

  LoopTransformConfig getDefault() const override {
    return LoopTransformConfig();
  }

  LoopTransformConfig getVal(const KnobConfig& Cfg) const override {
    auto It = Cfg.LoopCfg.find(getID());
    assert(It != Cfg.LoopCfg.end() && "Config is expected to be initialized here");
    return It->second;
  }

  void setVal(KnobConfig& Cfg, LoopTransformConfig LTC) override {
    Cfg.LoopCfg[getID()] = LTC;
  }

  std::string getName() override {
    return "Loop knob (id=" + std::to_string(getID()) + ")";
  }

};




}

#endif //CLANG_LOOPKNOB_H
