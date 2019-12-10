//
// Created by sebastian on 12.11.19.
//

#ifndef CLANG_TUNERDRIVER_H
#define CLANG_TUNERDRIVER_H

#include "../Driver.h"
#include "Debug.h"
#include "Tuner.h"

namespace clang {
namespace jit {

class TemplateArgKnob : public tuner::IntKnob {
public:
  TemplateArgKnob(unsigned Index, int Min, int Max, int Dflt)
      : tuner::IntKnob(Min, Max, Dflt, "Tunable Arg"), Index(Index){};

  unsigned getArgIndex() const { return Index; }

private:
  unsigned Index;
};

using VersionID = unsigned;

struct StatsTracker {
  bool HasBest{false};
  VersionID BestVersion;
  tuner::TimingStats BestStats;

  void update(unsigned ID, tuner::TimingStats Stats) {
    if (!HasBest || Stats.betterThan(BestStats)) {
      BestVersion = ID;
      BestStats = Stats;
      HasBest = true;
    }
  }
};

struct JITContext {

  JITContext() = default;

  //  JITContext(bool Emitted, std::unique_ptr<llvm::Module> Mod,  StringRef
  //  DeclName)
  //    : Emitted(Emitted), Mod(std::move(Mod)), DeclName(DeclName)
  //    {
  //
  //    }

  // Whether the function currently available in the JIT engine
  bool Emitted{false};
  // The unoptimized module
  std::unique_ptr<llvm::Module> Mod{nullptr};
  // Mangled function name
  SmallString<32> DeclName{""};
  // Symbols emitted during instantiation (also mangled) that need to be
  // replaced in the running module on recompilation.
  SmallVector<SmallString<16>, 16> ReplaceOnRecompilation{};
  // Version ID of the initial compilation. This module must not be deleted
  // since it contains the global definitions which all subsequent
  // recompilations depend on.
  VersionID PrimaryVersion{0};
  // Optimizer instance for this module
  std::unique_ptr<tuner::Optimizer> Opt{nullptr};
};

struct TunedCodeVersion {

  TunedCodeVersion() = default;

  TunedCodeVersion(VersionID ID, orc::VModuleKey ModKey, void *FPtr,
                   tuner::TimingGlobals Globals,
                   tuner::ConfigEvalRequest Request)
      : ID(ID), ModKey(ModKey), FPtr(FPtr), Globals(Globals),
        Request(std::move(Request)) {}

  tuner::TimingStats updateStats() {
    if (!Globals.Valid()) {
      return {};
    }
    auto CallCount = *Globals.CallCount;
    auto Mean = *Globals.MeanCycles;
    auto Var = *Globals.VarN / CallCount;
    auto NewStats = tuner::TimingStats(CallCount, Mean, Var);
    *Request.Stats = NewStats;
    return NewStats;
  }

  VersionID ID{0};
  orc::VModuleKey ModKey{0};
  void *FPtr{nullptr};
  tuner::TimingGlobals Globals;
  tuner::ConfigEvalRequest Request;
  // TimingStats Stats;
  // TODO: Place info about instantiation specific optimization here
};

struct JITTemplateInstantiation {

  static const VersionID InvalidID = 0;

  JITTemplateInstantiation() = default;

  JITTemplateInstantiation(JITContext &&Context, TunedCodeVersion Inst)
      : Context(std::move(Context)) {
    auto It = Instantiations.find(Inst.ID);
    assert(It == Instantiations.end() &&
           "Instantiation with same ID already exists!");
    Instantiations[Inst.ID] = Inst;
  }

  JITTemplateInstantiation(JITContext &&Context)
      : Context(std::move(Context)) {}

  void add(TunedCodeVersion Inst) {
    if (Inst.ID == InvalidID) {
      errs() << "Invalid instantiation!\n";
      return;
    }
    Instantiations[Inst.ID] = Inst;
    ActiveVersionID = Inst.ID;
  }

  TunedCodeVersion *getCurrentBest() {
    if (STracker.HasBest) {
      auto It = Instantiations.find(STracker.BestVersion);
      if (It != Instantiations.end())
        return &It->second;
    }
    return nullptr;
  }

  TunedCodeVersion *getActiveVersion() {
    if (ActiveVersionID == InvalidID)
      return nullptr;
    auto It = Instantiations.find(ActiveVersionID);
    if (It == Instantiations.end())
      return nullptr;
    return &It->second;
  }

  void updateStats() {
    auto Active = getActiveVersion();
    if (!Active)
      return;
    STracker.update(Active->ID, Active->updateStats());
    // We are interested in maximal performance, so we don't care about the
    // actual distribution of individual timing data within one template
    // specialization. Instead, we view the statistics of the best version as
    // representative of the whole set.
    if (STracker.HasBest)
      *Request.Stats = STracker.BestStats;
  }

  // Holds information that is needed for recompilation/reoptimization
  JITContext Context;

  tuner::ConfigEvalRequest Request;

  // Holds a list of instantiations
  llvm::DenseMap<VersionID, TunedCodeVersion> Instantiations;
  // Used to determine the best version
  StatsTracker STracker;
  // Holds the last compiled version ID
  VersionID ActiveVersionID{InvalidID};

  VersionID nextVersionID() { return VersionCounter++; }

private:
  VersionID VersionCounter{InvalidID + 1};
};

class TunableArgList {
public:
  TunableArgList(llvm::SmallVector<TemplateArgument, 8> BaseArgs)
      : BaseArgs(std::move(BaseArgs)) {}

  TunableArgList(const TunableArgList &) = delete;
  TunableArgList &operator=(const TunableArgList &) = delete;

  TunableArgList(TunableArgList &&) = default;

  void add(std::unique_ptr<TemplateArgKnob> Knob) {
    unsigned Idx = Knob->getArgIndex();
    Knobs[Idx] = std::move(Knob);
    TAKnobSet.add(Knobs[Idx].get());
  }

  tuner::KnobSet getKnobSet() { return TAKnobSet; }

  bool isTunable() { return TAKnobSet.count() > 0; }

  llvm::SmallVector<TemplateArgument, 8>
  getArgsForConfig(ASTContext &Ctx, const tuner::KnobConfig &Cfg) {
    llvm::SmallVector<TemplateArgument, 8> Args(BaseArgs);
    for (auto &It : Knobs) {
      unsigned Idx = It.first;
      auto Val = It.second->getVal(Cfg);
      APSInt APVal(32, Val >= 0);
      APVal = Val;
      Args[Idx] = TemplateArgument(Ctx, APVal,
                                   Args[Idx].getNonTypeTemplateArgumentType());
    }
    return Args;
  }

private:
  llvm::SmallVector<TemplateArgument, 8> BaseArgs;
  llvm::DenseMap<unsigned, std::unique_ptr<TemplateArgKnob>> Knobs;
  tuner::KnobSet TAKnobSet;
};

class BilevelTuningPolicy {
public:
  bool shouldChangeBaseParams(JITTemplateInstantiation &CurrentInst) {
    return CurrentInst.Instantiations.size() >= 5;
  }
};

struct ConfigMapInfo {
  static inline tuner::KnobConfig getEmptyKey() {
    auto Cfg = tuner::KnobConfig();
    Cfg.IntCfg[tuner::InvalidKnobID] = DenseMapInfo<int>::getEmptyKey();
    return Cfg;
  }

  static inline tuner::KnobConfig getTombstoneKey() {
    auto Cfg = tuner::KnobConfig();
    Cfg.IntCfg[tuner::InvalidKnobID] = DenseMapInfo<int>::getTombstoneKey();
    return Cfg;
  }

  static unsigned getHashValue(const tuner::KnobConfig &Cfg) {
    using llvm::hash_code;
    using llvm::hash_combine;
    using llvm::hash_combine_range;

    // TODO: This is painfully inefficient. Check if it's worth to optimize.
    std::vector<int> Keys(Cfg.IntCfg.size());
    for (auto It : Cfg.IntCfg) {
      Keys.push_back(It.first);
    }
    std::sort(Keys.begin(), Keys.end());
    hash_code h(1);
    for (auto Key : Keys) {
      h = hash_combine(Key, Cfg.IntCfg.find(Key)->second);
    }

    return (unsigned)h;
  }

  static bool isEqual(const tuner::KnobConfig &LHS,
                      const tuner::KnobConfig &RHS) {
    if (LHS.getNumDimensions() != RHS.getNumDimensions())
      return false;
    for (auto It : LHS.IntCfg) {
      auto It2 = RHS.IntCfg.find(It.first);
      if (It2 == RHS.IntCfg.end() || It.second != It2->second)
        return false;
    }
    return true;
    // TODO: This is just checking the integer data
  }
};

struct TemplateTuningData {

  using TAList = llvm::SmallVector<TemplateArgument, 8>;

  CompilerData &CD;
  unsigned Idx;

  // Template argument tuner.
  std::unique_ptr<tuner::Tuner> TATuner;
  TunableArgList TunableArgs;

  BilevelTuningPolicy Policy;

  // TODO: Change to another key type?
  llvm::DenseMap<tuner::KnobConfig, JITTemplateInstantiation, ConfigMapInfo>
      Specializations;

  tuner::KnobConfig ActiveConfig;
  bool Initialized;

  TemplateTuningData(CompilerData &CD, unsigned Idx,
                     std::unique_ptr<tuner::Tuner> TATuner,
                     TunableArgList TunableArgs)
      : CD(CD), Idx(Idx), TATuner(std::move(TATuner)),
        TunableArgs(std::move(TunableArgs)) {
    Initialized = false;
  }

  JITTemplateInstantiation &selectSpecialization() {
    bool ShouldChange = !Initialized;
    if (Initialized) {
      auto &TemplateInst = Specializations[ActiveConfig];
      TemplateInst.updateStats();
      ShouldChange = TunableArgs.isTunable() &&
                     Policy.shouldChangeBaseParams(TemplateInst);
    }
    if (ShouldChange) {
      auto EvalRequest = TATuner->generateNextConfig();
      ActiveConfig = EvalRequest.Cfg;

      auto Set = TunableArgs.getKnobSet();
      JIT_DEBUG(dbgs() << "Selected specialization:\n");
      JIT_DEBUG(tuner::KnobState(Set, ActiveConfig).dump());

      if (Specializations.find(ActiveConfig) == Specializations.end()) {
        auto TAs = TunableArgs.getArgsForConfig(*CD.Ctx, ActiveConfig);
        Specializations[ActiveConfig] = instantiate(TAs, EvalRequest);
      }
      Initialized = true;
    }
    return Specializations[ActiveConfig];
  }

  JITTemplateInstantiation* getSpecialization(const KnobConfig& Cfg) {
    auto It = Specializations.find(Cfg);
    if (It != Specializations.end())
      return &It->second;
    return nullptr;
  }

  llvm::Optional<std::pair<tuner::KnobConfig, JITTemplateInstantiation*>> computeOverallBest() {
    std::pair<tuner::KnobConfig, JITTemplateInstantiation*> CurrentBest;
    TimingStats BestStats;
    for (auto& It : Specializations) {
      auto& Inst = It.second;
      Inst.updateStats();
      auto Stats = Inst.getCurrentBest()->updateStats();
      if (Stats.betterThan(BestStats)) {
        BestStats = Stats;
        CurrentBest = {It.first, &Inst};
      }
    }
    if (BestStats.Valid()) {
      return CurrentBest;
    }
    return {};
  }

private:
  JITTemplateInstantiation instantiate(TAList TAs,
                                       tuner::ConfigEvalRequest Request) {
    //    for (auto& TA: TAs) {
    //      TA.dump();
    //      errs() << ", ";
    //    }
    TemplateInstantiationHelper InstHelper(CD, Idx);
    auto Name = InstHelper.instantiate(TAs);

    auto Mod = CD.createModule(Name);

    JIT_DEBUG(dumpModule(*Mod, "Initial module"));

    // TODO: Change module link time? This could be done for every
    // reoptimization, which would allow new definitions to
    //       be linked in too. However, this also introduces dependencies on
    //       other JIT modules.
    CD.linkInAvailableDefs(*Mod, false);

    JIT_DEBUG(dumpModule(*Mod, "Module after linking"));

    auto Opt = llvm::make_unique<tuner::Optimizer>(
        *CD.Diagnostics, *CD.HSOpts, CD.Invocation->getCodeGenOpts(),
        *CD.TargetOpts, *CD.Invocation->getLangOpts(),
        CD.CJ->getTargetMachine());
    Opt->init(Mod.get());

    JITContext JITCtx;
    JITCtx.DeclName = Name;
    JITCtx.Mod = llvm::CloneModule(*Mod);
    JITCtx.Opt = std::move(Opt);

    JITTemplateInstantiation TemplateInst(std::move(JITCtx));
    TemplateInst.Request = std::move(Request);
    return TemplateInst;
  }
};

class TunerDriver : public Driver {
public:
  explicit TunerDriver(CompilerData &CD) : Driver(CD){};

  InstData resolve(const ThisInstInfo &Inst, unsigned Idx) override;

private:
  llvm::DenseMap<InstInfo, std::unique_ptr<TemplateTuningData>, InstMapInfo>
      TuningDataMap;
};

} // namespace jit
} // namespace clang

#undef DEBUG_TYPE

#endif // CLANG_TUNERDRIVER_H
