//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_TUNER_H
#define CLANG_TUNER_H

#include <random>

#include "SimpleKnobs.h"
#include "LoopKnob.h"
#include "KnobSet.h"
#include "Util.h"

namespace tuner {

using TunerRNE = std::mt19937_64;

class Tuner {
public:
  virtual ~Tuner() {};
  virtual KnobConfig generateNextConfig() = 0;
};

template<typename RNETy>
struct GenRandomConfigFn: public KnobSetFn {

  explicit GenRandomConfigFn(RNETy& RNE) : RNE(RNE) {};

  void operator()(IntKnob& K) override {
    std::uniform_int_distribution<int> dist(K.min(), K.max());
    auto Val = dist(RNE);
    K.setVal(Cfg, Val);
  }

  void operator()(LoopKnob& K) override {
    auto LCfg = createRandomLoopConfig(RNE);
    K.setVal(Cfg, LCfg);
  }

  RNETy& RNE;
  KnobConfig Cfg;
};

template<typename RNETy>
KnobConfig createRandomConfig(RNETy& RNE, KnobSet& Set) {
  GenRandomConfigFn<RNETy> Fn(RNE);
  apply(Fn, Set);
  return Fn.Cfg;
}

class RandomTuner: public Tuner {
public:
  explicit RandomTuner(KnobSet& Knobs):
    Knobs(Knobs) {
    RNE = TunerRNE(util::genSeed());
  }


  KnobConfig generateNextConfig() override {
    CurrentConfig = createRandomConfig(RNE, Knobs);
    return CurrentConfig;
  }

private:
  KnobSet& Knobs;
  TunerRNE RNE;
  KnobConfig CurrentConfig;

};

}

#endif //CLANG_TUNER_H
