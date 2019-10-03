//
// Created by sebastian on 27.09.19.
//

#include "llvm/Analysis/LoopPass.h"
#include "KnobSet.h"
#include "LoopKnob.h"
#include "LoopMD.h"

using namespace llvm;

namespace tuner {

class LoopKnobCreator : public llvm::LoopPass {

private:
  unsigned LoopIDs = 0;
  KnobSet* Knobs;
public:
  static char ID;

  LoopKnobCreator()
      : LoopPass(ID), Knobs(nullptr) {};

  void setKnobs(KnobSet* KS) {
    this->Knobs = KS;
  }

  KnobSet* getKnobs() {
    return Knobs;
  }


  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {
    if (!Knobs) {
      errs() << "No KnobSet given!" << "\n";
      return false;
    }

    // TODO: Hello memory leak! Figure out who should own the knob.
    //       For now it's not a big deal since loop knobs should probably not be deleted before termination.
    LoopKnob* LK = new LoopKnob;
    Knobs->add(LK);

    //auto Name = LoopIDs++;
    auto LMD = assignLoopName(Loop, LK->getID());
    outs() << "Loop knob created with ID=" << LK->getID() << ":\n";

    return true;
  }

}; // end class

char LoopKnobCreator::ID = 0;
static RegisterPass <LoopKnobCreator> Register("loop-knob-creator",
                                         "Make loop knobs for tuning",
                                         false /* only looks at CFG*/,
                                         false /* analysis pass */);

llvm::Pass *createLoopKnobCreatorPass(KnobSet& KS) {
  auto LKC = new LoopKnobCreator();
  LKC->setKnobs(&KS);
  return LKC;
}

}