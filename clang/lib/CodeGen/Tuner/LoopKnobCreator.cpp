//
// Created by sebastian on 27.09.19.
//

#include "llvm/Analysis/LoopPass.h"
#include "KnobSet.h"
#include "LoopKnob.h"

using namespace llvm;

namespace tuner {

MDNode* getLoopID(Loop* Loop) {
  LLVMContext& Ctx = Loop->getHeader()->getContext();
  auto LoopID = Loop->getLoopID();
  if (!LoopID) {
    auto Dummy = MDNode::get(Ctx, {});
    LoopID = MDNode::get(Ctx, {Dummy});
    LoopID->replaceOperandWith(0, LoopID);
    Loop->setLoopID(LoopID);
  }
  return LoopID;
}

MDNode* createLoopName(Loop* Loop, unsigned Name) {
  LLVMContext& Ctx = Loop->getHeader()->getContext();
  auto LoopID = getLoopID(Loop);
  auto NameStr = MDString::get(Ctx, std::to_string(Name));
  auto LoopMD = MDNode::get(Ctx, {NameStr});
  MDNode::concatenate(LoopID, LoopMD);
  return LoopMD;
}

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

    auto Name = LoopIDs++;
    auto LMD = createLoopName(Loop, Name);

    // TODO: Hello memory leak! Figure out how should own the knob.
    //       For now it's not a big deal since loop knobs should probably not be deleted before termination.
    LoopKnob* LK = new LoopKnob(Name);

    Knobs->add(LK);

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