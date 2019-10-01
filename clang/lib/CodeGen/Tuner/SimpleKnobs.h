//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_KNOBS_H
#define CLANG_KNOBS_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Twine.h"
#include "Knob.h"
#include "KnobSet.h"

namespace tuner {


template<typename T>
class ScalarKnob: public Knob<T>{
public:

  virtual T min() const = 0;
  virtual T max() const = 0;

};

class IntKnob: public ScalarKnob<int> {
public:
  IntKnob(int Min, int Max, int Dflt, std::string Name) :
    MinVal(Min), MaxVal(Max), DefaultVal(Dflt), Name(std::move(Name))
  {}

  int min() const override {
    return MinVal;
  }

  int max() const override {
    return MaxVal;
  }

  int getDefault() const override {
    return DefaultVal;
  }

  int getVal(const KnobConfig& Cfg) const override {
    auto It = Cfg.IntCfg.find(getID());
    assert(It != Cfg.IntCfg.end() && "Config is expected to be initialized here");
    return It->second;
  }

  void setVal(KnobConfig& Cfg, int Val) override {
    Cfg.IntCfg[getID()] = Val;
  }

  std::string getName() override {
    return Name + " (id=" + std::to_string(getID()) + ")";
  }

private:
  int MinVal, MaxVal, DefaultVal;
  std::string Name;
};

namespace traits {
template<class T>
struct is_int_knob : public std::is_base_of<T, IntKnob>{};

template<class T>
using is_int_knob_t = typename is_int_knob<T>::value_type;
}


}

#endif //CLANG_KNOBS_H
