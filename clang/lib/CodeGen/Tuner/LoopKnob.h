//
// Created by sebastian on 27.09.19.
//

#ifndef CLANG_LOOPKNOB_H
#define CLANG_LOOPKNOB_H

#include "Knob.h"
#include "KnobSet.h"


namespace tuner {


class LoopKnob : public Knob<LoopTransformConfig> {
  LoopName Name;

public:
  LoopKnob(LoopName Name) :
    Name(Name)
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

  std::string getName() {
    return "Loop knob (id=" + std::to_string(getID()) + ")";
  }

};

}

#endif //CLANG_LOOPKNOB_H
