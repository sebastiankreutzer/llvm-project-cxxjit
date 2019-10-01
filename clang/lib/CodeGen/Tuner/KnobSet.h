//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_KNOBSET_H
#define CLANG_KNOBSET_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/raw_ostream.h"

#include "KnobDataTypes.h"
#include "Knob.h"

using namespace llvm;

namespace tuner {

class IntKnob;
class LoopKnob;

// A KnobSet is a collection of tuning knobs.
class KnobSet {
public:

  // NOTE: Cannot be inlined because of circular dependency issues
  void add(IntKnob* K);

  void removeIntKnob(KnobID ID) {
    IntKnobs.erase(ID);
  }

  void add(LoopKnob* K);

  void removeLoopKnob(KnobID ID) {
    LoopKnobs.erase(ID);
  }

  llvm::DenseMap<KnobID, IntKnob*> IntKnobs;
  llvm::DenseMap<KnobID, LoopKnob*> LoopKnobs;
};

// A Knob holds the data corresponding to a KnobSet.
struct KnobConfig {
//  Knob(KnobSet* KS) : Knobs(KS) {}
//
//  KnobSet* Knobs;

  llvm::DenseMap<KnobID, int> IntCfg;
  llvm::DenseMap <KnobID, LoopTransformConfig> LoopCfg;

};

// Encapsulates the configuration of a given KnobSet.
class KnobState {
public:
  KnobState(KnobSet& KS, KnobConfig& Cfg) :
    KS(KS), Config(Cfg) {}

  void dump() {
    dump(outs());
  }

  void dump(raw_ostream& OS);

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
