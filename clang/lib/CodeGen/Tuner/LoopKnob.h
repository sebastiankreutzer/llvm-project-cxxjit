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

  std::uniform_int_distribution<unsigned> VecWidthDist(LoopTransformConfig::VECTORIZE_WIDTH_MIN, LoopTransformConfig::VECTORIZE_WIDTH_MAX);
  Cfg.VectorizeWidthExp = VecWidthDist(RNE);

  // Interleave count should not be greater than the vectorization width
  const unsigned InterleaveMax = LoopTransformConfig::INTERLEAVE_COUNT_MAX;
  std::uniform_int_distribution<unsigned> InterleaveCountDist(LoopTransformConfig::INTERLEAVE_COUNT_MIN, std::min(Cfg.VectorizeWidthExp, InterleaveMax));
  Cfg.InterleaveCountExp = InterleaveCountDist(RNE);

  // Probably no reason why this should be disabled...
  Cfg.VectorizePredicateEnable = BiasedFlip(90);

  // Probably always beneficial?
  Cfg.DisableLICM = BiasedFlip(10);

  Cfg.DisableLICMVersioning = BiasedFlip(30);

  Cfg.Distribute = BiasedFlip(50);

  std::uniform_int_distribution<unsigned> UnrollDist(LoopTransformConfig::UNROLL_COUNT_MIN, LoopTransformConfig::UNROLL_COUNT_MAX);
  Cfg.UnrollCountExp = UnrollDist(RNE);

  Cfg.UnrollAndJam = BiasedFlip(50);

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
