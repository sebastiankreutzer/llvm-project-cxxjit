//
// Created by sebastian on 01.10.19.
//

#include <llvm/Support/FormatVariadic.h>
#include "llvm/Support/FormatAdapters.h"
#include "llvm/Analysis/LoopPass.h"
#include "LoopKnob.h"
#include "LoopMD.h"
#include "Passes.h"

unsigned InterleaveCount;
bool VectorizeEnable;
bool VectorizePredicateEnable;
unsigned VectorizeWidth;
bool DisableLICM;
bool DisableLICMVersioning;
bool Distribute;
namespace tuner {

static const char* DISABLE_NON_FORCED_TAG = "llvm.loop.disable_nonforced";
static const char* INTERLEAVE_COUNT_TAG = "llvm.loop.interleave.count";
static const char* VECTORIZE_ENABLE_TAG = "llvm.loop.vectorize.enable";
static const char* VECTORIZE_PREDICATE_ENABLE_TAG = "llvm.loop.vectorize.predicate.enable";
static const char* VECTORIZE_WIDTH_TAG = "llvm.loop.vectorize.width";
static const char* UNROLL_DISABLE_TAG = "llvm.loop.unroll.disable";
static const char* UNROLL_COUNT_TAG = "llvm.loop.unroll.count";
static const char* UNROLL_AND_JAM_DISABLE_TAG = "llvm.loop.unroll_and_jam.disable";
static const char* UNROLL_AND_JAM_COUNT_TAG = "llvm.loop.unroll_and_jam.count";
static const char* LICM_DISABLE_TAG = "llvm.licm.disable";
static const char* LICM_VERSIONING_DISABLE_TAG = "llvm.loop.licm_versioning.disable";
static const char* DISTRIBUTE_ENABLE_TAG = "llvm.loop.distribute.enable";


class ApplyLoopKnobs : public llvm::LoopPass {
private:
  KnobConfig* KnobCfg;
public:
  static char ID;

  ApplyLoopKnobs()
      : LoopPass(ID), KnobCfg(nullptr) {};

  void setKnobConfig(KnobConfig* Cfg) {
    this->KnobCfg = Cfg;
  }

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {
    if (!KnobCfg) {
      errs() << "No KnobConfig given!" << "\n";
      return false;
    }

    KnobID ID = getLoopName(Loop);
    if (ID == InvalidKnobID) {
      errs() << "Loop name not found: can't apply attributes (in function " << Loop->getHeader()->getParent()->getName() << ")\n";
      return false;
    }

    auto It = KnobCfg->LoopCfg.find(ID);
    if (It == KnobCfg->LoopCfg.end()) {
      errs() << "Loop is marked as tunable but no configuration found!\n";
      errs() << "In function " << Loop->getHeader()->getParent()->getName() << "\n";
      return false;
    }

    auto& Cfg = It->second;
    return applyConfig(Loop, Cfg);
  }
private:
  bool applyConfig(Loop* Loop, LoopTransformConfig& Cfg) {
    // TODO: Right now, we add all loop attributes directly to the main loop ID metadata.
    //       There is potential benefit in explicitly specifying the transformation order.

    auto LoopMD = Loop->getLoopID();
    if (!LoopMD)
      return false;

    // Vectorization
    // We only use explicit vectorization width, so a value of 0 will cause disabling instead of automatic determination.
    bool EnableVectorization = Cfg.getVectorizeWidth() > 1;

    LoopMD = addTaggedBool(LoopMD, VECTORIZE_ENABLE_TAG, EnableVectorization);
    if (EnableVectorization) {
      LoopMD = addTaggedInt32(LoopMD, VECTORIZE_WIDTH_TAG, Cfg.getVectorizeWidth());
      LoopMD = addTaggedInt32(LoopMD, INTERLEAVE_COUNT_TAG, Cfg.getInterleaveCount());
      LoopMD = addTaggedBool(LoopMD, VECTORIZE_PREDICATE_ENABLE_TAG, Cfg.VectorizePredicateEnable);
    }

    // Loop unrolling
    bool EnableUnrolling = Cfg.getUnrollCount() > 1;

    // NOTE: Behavior of "lllvm.loop.enable" according to the language reference:
    //       "[...] suggests that the loop should be fully unrolled if the trip count is known at compile time and
    //       partially unrolled if the trip count is not known at compile time."
    //       It is therefore probably better to not set this attribute directly, but use the "count" and "full" options instead.

    if (EnableUnrolling) {
      LoopMD = addTaggedInt32(LoopMD, UNROLL_COUNT_TAG, Cfg.getUnrollCount());
      // NOTE: We never set llvm.loop.unroll.full, see KnobDataTypes.h
      if (Cfg.UnrollAndJam) {
        // TODO: We use the same unroll count here, should they be handled separately?
        LoopMD = addTaggedInt32(LoopMD, UNROLL_AND_JAM_COUNT_TAG, Cfg.getUnrollCount());
      }
    } else {
      LoopMD = addTagMD(LoopMD, UNROLL_DISABLE_TAG);
      LoopMD = addTagMD(LoopMD, UNROLL_AND_JAM_DISABLE_TAG);
    }

    if (Cfg.DisableLICMVersioning) {
      LoopMD = addTagMD(LoopMD, LICM_VERSIONING_DISABLE_TAG);
    }

    LoopMD = addTaggedBool(LoopMD, DISTRIBUTE_ENABLE_TAG, Cfg.Distribute);

    if (Cfg.DisableLICM) {
      LoopMD = addTagMD(LoopMD, LICM_DISABLE_TAG);
    }

    if (Cfg.DisableNonForced) {
      LoopMD = addTagMD(LoopMD, DISABLE_NON_FORCED_TAG);
    }

    return true;
  }

}; // end class

char ApplyLoopKnobs::ID = 0;
static RegisterPass <ApplyLoopKnobs> Register("apply-loop-knob",
                                               "Apply tuning configuration to loop attributes",
                                               false /* only looks at CFG*/,
                                               false /* analysis pass */);

llvm::Pass *createApplyLoopKnobPass(KnobConfig& KnobCfg) {
  auto ALK = new ApplyLoopKnobs();
  ALK->setKnobConfig(&KnobCfg);
  return ALK;
}

void LoopTransformConfig::dump(llvm::raw_ostream& OS, unsigned Indent) const {
  auto I = llvm::formatv("{0}", llvm::fmt_repeat(" ", Indent));
  OS << I << DISABLE_NON_FORCED_TAG << ": " << DisableNonForced << "\n";
  OS << I << DISTRIBUTE_ENABLE_TAG << ": " << Distribute << "\n";
  OS << I << VECTORIZE_WIDTH_TAG << ": " << getVectorizeWidth() << "\n";
  OS << I << INTERLEAVE_COUNT_TAG << ": " << getInterleaveCount() << "\n";
  OS << I << UNROLL_COUNT_TAG << ": " << getUnrollCount() << "\n";
  OS << I << UNROLL_AND_JAM_DISABLE_TAG << ": " <<  !UnrollAndJam << "\n";
  OS << I << LICM_VERSIONING_DISABLE_TAG << ": " << DisableLICMVersioning << "\n";
  OS << I << LICM_DISABLE_TAG << ": " << DisableLICM << "\n";
  OS << I << VECTORIZE_PREDICATE_ENABLE_TAG << ": " << VectorizePredicateEnable << "\n";
}

llvm::raw_ostream& operator<<(llvm::raw_ostream& OS, const LoopTransformConfig& Cfg) {
  Cfg.dump(OS);
  return OS;
}

}
