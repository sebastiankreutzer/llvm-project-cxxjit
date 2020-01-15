//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_KNOBSET_H
#define CLANG_KNOBSET_H

#include "llvm/ADT/DenseMap.h"
#include "llvm/Support/raw_ostream.h"

#include "Knob.h"
#include "KnobDataTypes.h"

using namespace llvm;

namespace clang {
namespace jit {

class IntKnob;

class LoopKnob;

// A KnobSet is a collection of tuning knobs.
class KnobSet {
public:
  // NOTE: Cannot be inlined because of circular dependency issues
  void add(IntKnob *K);

  void removeIntKnob(KnobID ID) { IntKnobs.erase(ID); }

  void add(LoopKnob *K);

  void removeLoopKnob(KnobID ID) { LoopKnobs.erase(ID); }

  unsigned count() const { return IntKnobs.size() + LoopKnobs.size(); }

  unsigned countTunable() const;

  llvm::DenseMap<KnobID, IntKnob *> IntKnobs;
  llvm::DenseMap<KnobID, LoopKnob *> LoopKnobs;
};

// A Knob holds the data corresponding to a KnobSet.
struct KnobConfig {
  //  Knob(KnobSet* KS) : Knobs(KS) {}
  //
  //  KnobSet* Knobs;

  llvm::SmallDenseMap<KnobID, int, 8> IntCfg;
  llvm::SmallDenseMap<KnobID, LoopTransformConfig, 4> LoopCfg;

  unsigned getNumDimensions() const {
    return IntCfg.size() + LoopCfg.size() * LoopTransformConfig::NUM_PARAMS;
  }

  void addAll(KnobConfig& Other) {
    for (auto& It : Other.IntCfg) {
      IntCfg[It.first] = It.second;
    }
    for (auto& It : Other.LoopCfg) {
      LoopCfg[It.first] = It.second;
    }
  }
};

// Encapsulates the configuration of a given KnobSet.
class KnobState {
public:
  KnobState(const KnobSet &KS, KnobConfig &Cfg) : KS(KS), Config(Cfg) {}

  void dump() { dump(outs()); }

  void dump(raw_ostream &OS);

  const KnobSet &KS;
  KnobConfig &Config;
};

struct KnobSetFn {
  virtual void operator()(IntKnob &) = 0;

  virtual void operator()(LoopKnob &) = 0;
};

inline void apply(KnobSetFn &Fn, KnobSet &Set) {
  for (auto &K : Set.IntKnobs) {
    Fn(*K.second);
  }
  for (auto &K : Set.LoopKnobs) {
    Fn(*K.second);
  }
}

}
}

#endif // CLANG_KNOBSET_H
