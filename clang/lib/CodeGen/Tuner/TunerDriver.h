//
// Created by sebastian on 12.11.19.
//

#ifndef CLANG_TUNERDRIVER_H
#define CLANG_TUNERDRIVER_H

#include "../Driver.h"
#include "Debug.h"
#include "Tuner.h"
#include "Optimizer.h"
#include "TransformTreeOptimizer.h"

namespace clang {
namespace jit {


struct TunableNTTA {
  TunableNTTA(unsigned Index, long Min, long Max, long Dflt) : Index(Index), Param(Min, Max, Dflt, "Tunable NTTA"){
  }

  unsigned Index;
  SearchDim Param;
};

using VersionID = unsigned;

struct StatsTracker {
  bool HasBest{false};
  VersionID BestVersion;
  TimingStats BestStats;

  void update(unsigned ID, TimingStats Stats) {
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
  std::unique_ptr<Optimizer> Opt{nullptr};
};

struct TunedCodeVersion {

  TunedCodeVersion() = default;

  TunedCodeVersion(VersionID ID, orc::VModuleKey ModKey, void *FPtr,
                   TimingGlobals Globals,
                   ConfigEval Request)
      : ID(ID), ModKey(ModKey), FPtr(FPtr), Globals(Globals),
        Request(std::move(Request)) {}

  TimingStats updateStats() {
    if (!Globals.Valid()) {
      return {};
    }
    auto CallCount = *Globals.CallCount;
    auto Mean = *Globals.MeanCycles;
    auto Var = *Globals.VarN / CallCount;
    auto NewStats = TimingStats(CallCount, Mean, Var);
    *Request.Stats = NewStats;
    return NewStats;
  }

  VersionID ID{0};
  orc::VModuleKey ModKey{0};
  void *FPtr{nullptr};
  TimingGlobals Globals;
  ConfigEval Request;
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

  ConfigEval Request;

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
      : BaseArgs(std::move(BaseArgs)), Space(std::make_unique<SearchSpace>()) {}

  TunableArgList(const TunableArgList &) = delete;
  TunableArgList &operator=(const TunableArgList &) = delete;

  TunableArgList(TunableArgList &&) = default;

  void add(TunableNTTA TA) {
    unsigned Idx = TA.Index;
    Space->addDim(std::move(TA.Param));
    TunableDimensions[Idx] = Space->getNumDimensions() - 1;
  }

//  KnobSet getKnobSet() { return TAKnobSet; }

  SearchSpace& getSearchSpace() {
    return *Space;
  }

  bool isTunable() { return Space->getNumDimensions() > 0; }

  llvm::SmallVector<TemplateArgument, 8>
  getArgsForConfig(ASTContext &Ctx, const ParamConfig& Cfg) {
    llvm::SmallVector<TemplateArgument, 8> Args(BaseArgs);
    for (auto &It : TunableDimensions) {
      unsigned Idx = It.first;
      auto& Dim = Space->getDim(It.second);
      auto Val = Cfg[It.second];
      if (!Val.getIntVal()) {
        report_fatal_error("Expected int value");
      }
      auto IntVal = cantFail(Val.getIntVal());
      APSInt APVal(32, IntVal >= 0);
      APVal = IntVal;
      Args[Idx] = TemplateArgument(Ctx, APVal,
                                   Args[Idx].getNonTypeTemplateArgumentType());
    }
    return Args;
  }

private:
  llvm::SmallVector<TemplateArgument, 8> BaseArgs;

  std::unique_ptr<SearchSpace> Space;

  // Maps from parameter index to the dimension in the search space.
  llvm::DenseMap<unsigned, unsigned> TunableDimensions;
};

class BilevelTuningPolicy {
public:
  bool shouldChangeBaseParams(JITTemplateInstantiation &CurrentInst) {
    return CurrentInst.Instantiations.size() >= 5;
  }
};

struct HashableConfig {
  friend struct ParamConfigMapInfo;
  HashableConfig() {
  }

  HashableConfig(ParamConfig Config) : Config(std::move(Config)) {
  }

  ParamConfig& get() {
    return Config;
  }

  const ParamConfig& get() const {
    return Config;
  }


  ParamConfig Config;
private:
  int EmptyOrTombstone{1};
};

struct ParamConfigMapInfo {
  static inline HashableConfig getEmptyKey() {
    HashableConfig Cfg;
    Cfg.EmptyOrTombstone = DenseMapInfo<int>::getEmptyKey();
    return Cfg;
  }

  static inline HashableConfig getTombstoneKey() {
    HashableConfig Cfg;
    Cfg.EmptyOrTombstone = DenseMapInfo<int>::getTombstoneKey();
    return Cfg;
  }

  static unsigned getHashValue(const HashableConfig& Cfg) {
    using llvm::hash_code;
    using llvm::hash_combine;
    using llvm::hash_combine_range;

    hash_code h(Cfg.EmptyOrTombstone);
    for (unsigned I = 0; I < Cfg.get().size(); I++) {
      if (Cfg.get()[I].Type == ParamType::INT)
        h = hash_combine(h, cantFail(Cfg.get()[I].getIntVal()));
    }

    return (unsigned)h;

  }

  static bool isEqual(const HashableConfig& LHS, const HashableConfig& RHS) {
    if (LHS.EmptyOrTombstone != RHS.EmptyOrTombstone)
      return false;
    if (LHS.get().Space != RHS.get().Space)
      return false;
    if (LHS.get().size() != RHS.get().size()) {
      LHS.get().dump();
      outs() << "\n";
      RHS.get().dump();
    }
    assert(LHS.get().size() == RHS.get().size() && "Same space but different number of values");
    for (unsigned I = 0; I < LHS.get().size(); I++) {
      if (LHS.get()[I].Type == ParamType::INT) {
        assert(RHS.get()[I].Type == ParamType::INT && "Same space but different types");
        if (cantFail(LHS.get()[I].getIntVal()) != cantFail(RHS.get()[I].getIntVal()))
          return false;
      }
    }
    return true;
  }
};


struct TemplateTuningData {

  using TAList = llvm::SmallVector<TemplateArgument, 8>;

  CompilerData &CD;
  unsigned Idx;

  // Template argument tuner.
  std::unique_ptr<Tuner> TATuner;
  TunableArgList TunableArgs;

  BilevelTuningPolicy Policy;

  // TODO: Change to another key type?
  llvm::DenseMap<HashableConfig, JITTemplateInstantiation, ParamConfigMapInfo>
      Specializations;

  ParamConfig ActiveConfig;
  bool Initialized;

  TemplateTuningData(CompilerData &CD, unsigned Idx,
                     std::unique_ptr<Tuner> TATuner,
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

      ConfigEval Eval;
      if (TATuner) {
        Eval = TATuner->generateNextConfig();
      } else {
        // No tunable args
        Eval = ConfigEval();
      }
      ActiveConfig = Eval.Config;

//      auto Set = TunableArgs.getKnobSet();
      JIT_DEBUG(dbgs() << "Selected specialization:\n");
      // TODO
//      JIT_DEBUG(KnobState(Set, ActiveConfig).dump());

      Initialized = true;
      auto It = Specializations.find(ActiveConfig);
      if (It == Specializations.end()) {
        auto TAs = TunableArgs.getArgsForConfig(*CD.Ctx, ActiveConfig);
        Specializations[ActiveConfig] = instantiate(TAs, Eval);
      }

    }
    return Specializations[ActiveConfig];
  }
  
  bool hasTunableArgs() {
    return TunableArgs.isTunable();
  }

  JITTemplateInstantiation* getSpecialization(const ParamConfig& Cfg) {
    auto It = Specializations.find(Cfg);
    if (It != Specializations.end())
      return &It->second;
    return nullptr;
  }

  llvm::Optional<std::pair<ParamConfig, JITTemplateInstantiation*>> computeOverallBest() {
    std::pair<ParamConfig, JITTemplateInstantiation*> CurrentBest;
    TimingStats BestStats;
    for (auto& It : Specializations) {
      auto& Inst = It.second;
      Inst.updateStats();
      auto Stats = Inst.getCurrentBest()->updateStats();
      if (Stats.betterThan(BestStats)) {
        BestStats = Stats;
        CurrentBest = {It.first.get(), &Inst};
      }
    }
    if (BestStats.Valid()) {
      return CurrentBest;
    }
    return {};
  }

private:
  JITTemplateInstantiation instantiate(TAList TAs,
                                       ConfigEval Request) {
    //    for (auto& TA: TAs) {
    //      TA.dump();
    //      errs() << ", ";
    //    }
    TemplateInstantiationHelper InstHelper(CD, Idx);
    auto Name = InstHelper.instantiate(TAs);

    auto Mod = CD.createModule(Name);

    JIT_DEBUG(util::dumpModule(*Mod, "Initial module"));

    // TODO: Change module link time? This could be done for every
    // reoptimization, which would allow new definitions to
    //       be linked in too. However, this also introduces dependencies on
    //       other JIT modules.
    //CD.linkInAvailableDefs(*Mod, false); // FIXME: Uncomment this!

    JIT_DEBUG(util::dumpModule(*Mod, "Module after linking"));

    // TODO: Make this configurable
#define TRANSFORM_TREE_OPT
#ifdef TRANSFORM_TREE_OPT
    auto Opt = std::make_unique<TransformTreeOptimizer>(
        *CD.Diagnostics, *CD.HSOpts, CD.Invocation->getCodeGenOpts(),
        *CD.TargetOpts, *CD.Invocation->getLangOpts(),
        CD.CJ->getTargetMachine());
#else
    auto Opt = std::make_unique<StaticOptimizer>(
        *CD.Diagnostics, *CD.HSOpts, CD.Invocation->getCodeGenOpts(),
        *CD.TargetOpts, *CD.Invocation->getLangOpts(),
        CD.CJ->getTargetMachine());
#endif
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

  void* finishTuning(const InstInfo& Inst);

private:
  llvm::DenseMap<InstInfo, std::unique_ptr<TemplateTuningData>, InstMapInfo>
      TuningDataMap;
};

} // namespace jit
} // namespace clang

#undef DEBUG_TYPE

#endif // CLANG_TUNERDRIVER_H
