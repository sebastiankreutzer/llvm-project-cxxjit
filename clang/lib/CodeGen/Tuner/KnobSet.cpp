//
// Created by sebastian on 30.09.19.
//

#include "KnobSet.h"
#include "LoopKnob.h"
#include "SimpleKnobs.h"

namespace tuner {

void KnobSet::add(IntKnob *K) { IntKnobs[K->getID()] = K; }

void KnobSet::add(LoopKnob *K) { LoopKnobs[K->getID()] = K; }

void KnobState::dump(raw_ostream &OS) {
  for (auto &IK : KS.IntKnobs) {
    OS << IK.second->getName() << ": " << IK.second->getVal(Config) << "\n";
  }
  for (auto &LK : KS.LoopKnobs) {
    OS << LK.second->getName() << ":\n";
    LK.second->getVal(Config).dump(OS, 3);
  }
}

} // namespace tuner
