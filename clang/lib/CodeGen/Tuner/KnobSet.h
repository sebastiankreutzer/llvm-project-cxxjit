//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_KNOBSET_H
#define CLANG_KNOBSET_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/raw_ostream.h"

#include "Knobs.h"

using namespace llvm;

namespace tuner {

class KnobSet {
public:

  void add(IntKnob* K) {
    IntKnobs[K->getID()] = K;
  }

  void remove(KnobID ID) {
    IntKnobs.erase(ID);
  }

  llvm::DenseMap<KnobID, IntKnob*> IntKnobs;
};

class KnobState {
public:
  KnobState(KnobSet& KS, KnobConfig Cfg) :
    KS(KS), Config(Cfg) {}

  void dump() {
    dump(outs());
  }

  void dump(raw_ostream& OS) {
    for (auto& IK : KS.IntKnobs) {
      OS << IK.second->getName() << ": " << IK.second->getVal(Config) << "\n";
    }
  }

  KnobSet& KS;
  KnobConfig& Config;
};



struct KnobSetFn {
  virtual void operator()(IntKnob& knob) = 0;
};

inline void apply(KnobSetFn& Fn, KnobSet& Set) {
  for (auto& K : Set.IntKnobs) {
    Fn(*K.second);
  }
}

}

#endif //CLANG_KNOBSET_H
