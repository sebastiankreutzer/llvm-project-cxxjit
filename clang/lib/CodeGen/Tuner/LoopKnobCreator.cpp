//
// Created by sebastian on 27.09.19.
//

#include "Debug.h"
#include "KnobSet.h"
#include "LoopKnob.h"
#include "LoopMD.h"
#include "llvm/Analysis/LoopPass.h"

using namespace llvm;

namespace clang {
namespace jit {

class LoopKnobCreator : public llvm::LoopPass {

private:
  unsigned LoopIDs = 0;
  KnobSet *Knobs;

  unsigned getMaxDepth(Loop *L) {
    auto &SubLoops = L->getSubLoops();
    unsigned Depth = 0;
    for (auto *SL : SubLoops) {
      Depth = std::max(Depth, getMaxDepth(SL));
    }
    return Depth + 1;
  }

  LoopKnob *createKnob(Loop *L) {
    // TODO: Can we somehow get the real variable name?
    auto Name = L->getCanonicalInductionVariable() && L->getCanonicalInductionVariable()->hasName()
                ? L->getCanonicalInductionVariable()->getName() : "unknown";
    // TODO: Hello memory leak! Figure out who should own the knob.
    //       For now it's not a big deal since loop knobs should probably not be
    //       deleted before termination.
    LoopKnob *LK = new LoopKnob(Name);
    Knobs->add(LK);

    bool IsInnermost = L->getSubLoops().empty();
    unsigned MaxDepth = getMaxDepth(L);

    // Here, the min/max/default values are set for each attribute.
    // Attributes that are note explicitly set here use the global defaults.

    // Distribute
    if (IsInnermost) {
      LK->setAttributeBounds(LoopTransformConfig::DISTRIBUTE, 0, 1, 1);
    } else {
      LK->disableTuning(LoopTransformConfig::DISTRIBUTE, 0);
    }
    LK->disableTuning(LoopTransformConfig::DISTRIBUTE, 0); // TODO: Always disable for now, does not seem to do anything

    // Vectorization
    // TODO: Look up supported SIMD width
    if (IsInnermost) {
      LK->setAttributeBounds(LoopTransformConfig::VECTORIZE_WIDTH, 0, 3, 0);
      LK->setAttributeBounds(LoopTransformConfig::INTERLEAVE_COUNT, 0, 3, 0);
      LK->setAttributeBounds(LoopTransformConfig::VECTORIZE_PREDICATE_ENABLE, 0, 1, 1);
    } else {
      LK->disableTuning(LoopTransformConfig::VECTORIZE_WIDTH, 0);
      LK->disableTuning(LoopTransformConfig::INTERLEAVE_COUNT, 0);
      LK->disableTuning(LoopTransformConfig::VECTORIZE_PREDICATE_ENABLE, 0);
    }

    // Unroll-and-jam
    // Enabled only for the second and third innermost loops.
    if (IsInnermost || MaxDepth > 3) {
      LK->disableTuning(LoopTransformConfig::UNROLL_AND_JAM, 0);
    }

    // Unrolling
    // Enable only for the two innermost loops
    if (MaxDepth <= 2) {
      LK->setAttributeBounds(LoopTransformConfig::UNROLL_COUNT, 0, 5, 0);
    } else {
      LK->disableTuning(LoopTransformConfig::UNROLL_COUNT, 0);
    }

    // auto Name = LoopIDs++;
    auto LMD = assignLoopName(L, LK->getID());
    JIT_INFO(dbgs() << "Loop knob created with ID=" << LK->getID() << "\n");

    for (auto &SubLoop : L->getSubLoops()) {
      LK->addSubLoopKnob(createKnob(SubLoop));
    }
    return LK;
  }

public:
  static char ID;

  LoopKnobCreator() : LoopPass(ID), Knobs(nullptr) {};

  void setKnobs(KnobSet *KS) { this->Knobs = KS; }

  KnobSet *getKnobs() { return Knobs; }

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {
    if (!Knobs) {
      errs() << "No KnobSet given!"
             << "\n";
      return false;
    }

    if (Loop->getHeader()->getParent()->isDeclarationForLinker()) {
      // We don't want to consider loops that are marked available_externally
      return false;
    }

    if (!Loop->getParentLoop()) {
      createKnob(Loop);
      return true;
    }
    return false;

  }

}; // end class

char LoopKnobCreator::ID = 0;
static RegisterPass<LoopKnobCreator> Register("loop-knob-creator",
                                              "Make loop knobs for tuning",
                                              false /* only looks at CFG*/,
                                              false /* analysis pass */);

llvm::Pass *createLoopKnobCreatorPass(KnobSet &KS) {
  auto LKC = new LoopKnobCreator();
  LKC->setKnobs(&KS);
  return LKC;
}

}
}