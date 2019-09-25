//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_KNOBS_H
#define CLANG_KNOBS_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/ADT/Twine.h"

namespace tuner {

template<typename ValT>
class Knob;

class KnobRegistry;

using KnobID = unsigned;

struct KnobConfig {
//  KnobConfig(KnobSet* KS) : Knobs(KS) {}
//
//  KnobSet* Knobs;

  llvm::DenseMap<KnobID, int> IntCfg;

};

class KnobBase {
public:
  inline KnobBase();
  virtual ~KnobBase() {};

  KnobID getID() const {
    return ID;
  }

  friend struct KnobRegistry;

private:
  KnobID ID;
};

struct KnobRegistry {
  static void registerKnob(KnobBase* K) {
    static KnobID Counter = 1;
    K->ID = Counter++;
  }
};

KnobBase::KnobBase() {
  ID = 0;
  KnobRegistry::registerKnob(this);
}

template<typename ValT>
class Knob: public KnobBase {
public:

  using ValTy = ValT;

  //virtual void applyConfig(const KnobConfig&) = 0;

  virtual ValTy getDefault() const = 0;
  virtual ValTy getVal(const KnobConfig& Cfg) const = 0;
  virtual void setVal(KnobConfig& Cfg, ValTy) = 0;

  virtual std::string getName() {
    return "Knob (id=" + std::to_string(getID()) + ")";
  }

};


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
