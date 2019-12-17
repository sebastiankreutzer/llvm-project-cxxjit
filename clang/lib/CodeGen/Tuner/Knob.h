//
// Created by sebastian on 30.09.19.
//

#ifndef CLANG_KNOBCONFIG_H
#define CLANG_KNOBCONFIG_H

#include <string>

namespace clang {
namespace jit {

using KnobID = unsigned;

constexpr KnobID InvalidKnobID = 0;

template<typename ValT>
class Knob;

struct KnobRegistry;
struct KnobConfig;

class KnobBase {
public:
  inline KnobBase();

  // Copies are not allowed to avoid confusion with IDs.
  // Also, we don't want two knobs with the same ID to be able to have different
  // properties.
  KnobBase(const KnobBase &) = delete;

  KnobBase &operator=(const KnobBase &) = delete;

  KnobBase(KnobBase &&Other) {
    ID = Other.ID;
    Other.ID = InvalidKnobID;
  }

  KnobBase &operator=(KnobBase &&Other) {
    if (this != &Other) {
      ID = Other.ID;
      Other.ID = InvalidKnobID;
    }
    return *this;
  }

  virtual ~KnobBase() {};

  KnobID getID() const { return ID; }

  friend struct KnobRegistry;

private:
  KnobID ID;
};

struct KnobRegistry {
  static void registerKnob(KnobBase *K) {
    static KnobID Counter = 1;
    K->ID = Counter++;
  }
};

KnobBase::KnobBase() {
  ID = 0;
  KnobRegistry::registerKnob(this);
}

template<typename ValT>
class Knob : public KnobBase {
public:
  using ValTy = ValT;

  // virtual void applyConfig(const Knob&) = 0;

  virtual ValTy getDefault() const = 0;

  virtual ValTy getVal(const KnobConfig &Cfg) const = 0;

  virtual void setVal(KnobConfig &Cfg, ValTy) = 0;

  virtual std::string getName() const {
    return "Knob (id=" + std::to_string(getID()) + ")";
  }
};

}
}

#endif // CLANG_KNOBCONFIG_H
