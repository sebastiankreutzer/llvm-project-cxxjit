//
// Created by sebastian on 30.09.19.
//

#ifndef CLANG_KNOBCONFIG_H
#define CLANG_KNOBCONFIG_H

namespace tuner {

using KnobID = unsigned;

template<typename ValT>
class Knob;
class KnobRegistry;
struct KnobConfig;

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

  //virtual void applyConfig(const Knob&) = 0;

  virtual ValTy getDefault() const = 0;
  virtual ValTy getVal(const KnobConfig& Cfg) const = 0;
  virtual void setVal(KnobConfig& Cfg, ValTy) = 0;

  virtual std::string getName() {
    return "Knob (id=" + std::to_string(getID()) + ")";
  }

};

}

#endif //CLANG_KNOBCONFIG_H
