//
// Created by sebastian on 12.11.19.
//

#include "TunerDriver.h"

#include "Debug.h"
#include "Tuners.h"
#include "clang/CodeGen/Tuning.h"


namespace {

using namespace clang;
using namespace clang::jit;

struct PtrData {
  PtrData(TunerDriver* Driver, const InstInfo& Inst) : Driver(Driver), Inst(Inst) {

  }

  TunerDriver* Driver;
  InstInfo Inst;
};

DenseMap<void*, PtrData> PtrToInstInfo;
}


namespace clang {
namespace jit {


namespace {
void printReport(JITTemplateInstantiation &TemplateInst) {
  auto &FName = TemplateInst.Context.DeclName;

  outs() << "JIT Timing Report:\n";
  auto Header = formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}",
                        fmt_align("Name", AlignStyle::Center, FName.size()),
                        "ID", "#Called", "Mean", "RSD", "RSE")
                    .str();
  outs() << Header << "\n";
  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
  // Iterate over IDs to display in order. Not very robust, but works for
  // debugging.
  for (unsigned i = 1; i <= TemplateInst.Instantiations.size(); i++) {
    auto It = TemplateInst.Instantiations.find(i);
    if (It == TemplateInst.Instantiations.end())
      continue;
    auto &Inst = It->second;
    auto &PG = Inst.Globals;
    if (!PG.Valid()) {
      errs() << FName.str()
             << " Performance tracking globals have not been registered "
                "correctly\n";
      continue;
    }
    auto Stats = Inst.updateStats();
    if (!Stats.valid()) {
      outs() << FName.str() << " No data collected\n";
      return;
    }
    outs() << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n", FName,
                      Inst.ID, Stats.N, formatv("{0:f1}", Stats.Mean),
                      formatv("{0:p}", Stats.getRSD()),
                      formatv("{0:p}", Stats.getRelativeStdErr()));
  }
  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
  auto BestID = TemplateInst.getCurrentBest()->ID;
  auto& Best = TemplateInst.Instantiations[BestID];
  // TODO: We assume here that the first version is the baseline, make this
  // explicit
  auto& BaseLine = TemplateInst.Instantiations[1];
  outs() << formatv("Best version: {0} ({1:p} speedup)\n", BestID,
                    BaseLine.updateStats().Mean / Best.updateStats().Mean -
                        1.0);
  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
}

void printShortReport(TemplateTuningData &Data, llvm::raw_ostream &OS = dbgs()) {
  if (!Data.Initialized || Data.Specializations.empty()) {
    return;
  }
  auto OverallBest = Data.computeOverallBest();
  if (!OverallBest) {
    return;
  }

  auto Header = formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}",
                        fmt_align("Name", AlignStyle::Center, OverallBest.getValue().second->Context.DeclName.size()), "ID",
                        "#Called", "Mean", "RSD", "RSE").str();
  OS << "Best Configuration:\n";
  OS << formatv("{0}\n", fmt_repeat("=", Header.size()));
  OS << Header << "\n";
  OS << formatv("{0}\n", fmt_repeat("-", Header.size()));


  auto BestSpecialization = OverallBest.getValue().second;
  auto BestVersion = BestSpecialization->getCurrentBest();
  auto BestStats = BestVersion->updateStats();
  auto& BaseLine = BestSpecialization->Instantiations[1];
  OS << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                BestSpecialization->Context.DeclName, BestVersion->ID, BestStats.N,
                formatv("{0:f1}", BestStats.Mean),
                formatv("{0:p}", BestStats.getRSD()),
                formatv("{0:p}", BestStats.getRelativeStdErr()));
  OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
  OS << formatv("Speedup: ({0:p} speedup)\n", BaseLine.updateStats().Mean / BestStats.Mean - 1.0);
  OS << formatv("{0}\n", fmt_repeat("-", Header.size()));

  OS << formatv("{0}\n", fmt_repeat("=", Header.size()));

}

void printFullReport(TemplateTuningData &Data, llvm::raw_ostream &OS = dbgs()) {
  if (!Data.Initialized || Data.Specializations.empty()) {
    return;
  }

  auto OverallBest = Data.computeOverallBest();

  auto FirstName = Data.Specializations.begin()->second.Context.DeclName;
  auto MaxNameLen = FirstName.size() + 4; // This does not always work.

  auto Header = formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}",
                        fmt_align("Name", AlignStyle::Center, MaxNameLen), "ID",
                        "#Called", "Mean", "RSD", "RSE")
      .str();
  OS << Header << "\n";
  OS << formatv("{0}\n", fmt_repeat("=", Header.size()));
  for (auto &It : Data.Specializations) {
    auto &TemplateInst = It.second;
    auto FName = TemplateInst.Context.DeclName;
    OS << FName << "\n";


    // Iterate over IDs to display in order. Not very robust, but works for
    // debugging.
    for (unsigned i = 1; i <= TemplateInst.Instantiations.size(); i++) {
      auto It = TemplateInst.Instantiations.find(i);
      if (It == TemplateInst.Instantiations.end())
        continue;
      auto &Inst = It->second;
      auto &PG = Inst.Globals;
      if (!PG.Valid()) {
        errs() << "Performance tracking globals have not been registered "
                  "correctly\n";
        continue;
      }
      auto Stats = Inst.updateStats();
      if (!Stats.valid()) {
        OS << "No data collected\n";
        return;
      }
      OS << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                    fmt_repeat(" ", MaxNameLen), Inst.ID, Stats.N,
                    formatv("{0:f1}", Stats.Mean),
                    formatv("{0:p}", Stats.getRSD()),
                    formatv("{0:p}", Stats.getRelativeStdErr()));
    }

    OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
    auto BestID = TemplateInst.getCurrentBest()->ID;
    auto& Best = TemplateInst.Instantiations[BestID];
    // TODO: We assume here that the first version is the baseline, make this
    // explicit
    auto& BaseLine = TemplateInst.Instantiations[1];
    OS << formatv("Best version: {0} ({1:p} speedup)\n", BestID,
                  BaseLine.updateStats().Mean / Best.updateStats().Mean - 1.0);
    OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
  }
  OS << formatv("{0}\n", fmt_repeat("=", Header.size()));

  if (OverallBest) {
    auto& BestSpecialization = OverallBest.getValue().second;
    auto BestVersion = BestSpecialization->getCurrentBest();
    auto BestStats = BestVersion->updateStats();
    auto& BaseLine = BestSpecialization->Instantiations[1];
    OS << "Overall best: \n";
    OS << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                  BestSpecialization->Context.DeclName, BestVersion->ID, BestStats.N,
                        formatv("{0:f1}", BestStats.Mean),
                        formatv("{0:p}", BestStats.getRSD()),
                        formatv("{0:p}", BestStats.getRelativeStdErr()));
    OS << formatv("Speedup: ({0:p} speedup)\n", BaseLine.updateStats().Mean / BestStats.Mean - 1.0);
    OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
    OS << "Found configuration: \n";
    OS << formatv("{0}\n", fmt_repeat("=", Header.size()));
  }
}

void printReport(TemplateTuningData &Data) {
  if (!Data.Initialized || Data.Specializations.empty()) {
    return;
  }
  auto FName = Data.Specializations.begin()->second.Context.DeclName;
  outs() << "JIT Timing Report:\n";
  auto Header = formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}",
                        fmt_align("Name", AlignStyle::Center, FName.size()),
                        "ID", "#Called", "Mean", "RSD", "RSE")
                    .str();
  outs() << Header << "\n";
  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
  // Iterate over IDs to display in order. Not very robust, but works for
  // debugging.
  for (auto &It : Data.Specializations) {
    It.second.updateStats();
    if (!It.second.getCurrentBest())
      continue;
    auto &Version = *It.second.getCurrentBest();

    auto &PG = Version.Globals;
    if (!PG.Valid()) {
      errs() << FName.str()
             << " Performance tracking globals have not been registered "
                "correctly\n";
      continue;
    }
    auto Stats = Version.updateStats();
    if (!Stats.valid()) {
      outs() << FName.str() << " No data collected\n";
      return;
    }
    outs() << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                      It.second.Context.DeclName, Version.ID, Stats.N,
                      formatv("{0:f1}", Stats.Mean),
                      formatv("{0:p}", Stats.getRSD()),
                      formatv("{0:p}", Stats.getRelativeStdErr()));
  }
  //  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
  //  auto BestID = TemplateInst.getCurrentBest()->ID;
  //  auto Best = TemplateInst.Instantiations[BestID];
  //  // TODO: We assume here that the first version is the baseline, make this
  //  explicit auto BaseLine = TemplateInst.Instantiations[1]; outs() <<
  //  formatv("Best version: {0} ({1:p} speedup)\n", BestID,
  //  BaseLine.updateStats().Mean / Best.updateStats().Mean - 1.0);
  outs() << formatv("{0}\n", fmt_repeat("=", Header.size()));
}

} // namespace

InstData TunerDriver::resolve(const ThisInstInfo &Inst, unsigned Idx) {
  TemplateTuningData *TTD;
  auto It = TuningDataMap.find_as(Inst);
  if (It == TuningDataMap.end()) {
    JIT_DEBUG(dbgs() << "Resolving JIT template " << Inst.InstKey << "\n");

    // TODO: move to fdata (?)
    SmallVector<TunableNTTA, 4> TunableNTTAs;

    llvm::SmallVector<TemplateArgument, 8> BaseArgs;

    TemplateInstantiationHelper InstHelper(CD, Idx);
    InstHelper.processTemplateArgs(
        Inst.NTTPValues, Inst.TypeStrings, BaseArgs,
        [this, &TunableNTTAs](QualType CanonType, unsigned Pos,
                         const SmallVectorImpl<uint64_t> &IntWords)
            -> Optional<TemplateArgument> {
          // errs() << "Instantiating NTTA with " << Size << " bytes: \n";
          // CanonType.dump();
          // Fld->dump();
          // errs() << "Data: " << IntVal << "\n";

          JIT_DEBUG(dbgs() << "Processing argument of type: "
                           << CanonType.getAsString() << "\n");

          if (CanonType.getAsString() ==
              "struct clang::jit::tunable_range<int>") {
            auto FieldDecl = CanonType.getTypePtr()->getAsRecordDecl();
            auto TemplateDecl =
                dyn_cast<ClassTemplateSpecializationDecl>(FieldDecl);
            assert(TemplateDecl &&
                   "clang::jit::tunable_range must be template");
            // TODO: How to handle template arg other than int? Should we even
            // use templated range?
            auto Range = *reinterpret_cast<const jit::tunable_range<int> *>(
                &IntWords[0]);
            JIT_DEBUG(dbgs() << "Type is tunable: Range is [" << Range.Min
                             << "; " << Range.Max << "]\n");

            llvm::APSInt SIntVal(32); // TODO allow varying bit widths
            SIntVal = Range.Min;

            TunableNTTAs.emplace_back(Pos, Range.Min, Range.Max, Range.Min);

            return TemplateArgument(
                *CD.Ctx, SIntVal,
                CD.Ctx->getIntTypeForBitwidth(
                    32,
                    false)); // TODO: Check if expected template params matches
          }
          return {};
        });

    JIT_INFO(outs() << "Looking for tunable template arguments..." << "\n");
    TunableArgList TAL(BaseArgs);
    for (auto& NTTA : TunableNTTAs) {
      TAL.add(NTTA);
    }

    std::unique_ptr<Tuner> TATuner;
    if (TAL.isTunable()) {
      TATuner = createTuner(loadSearchAlgoEnv(), TAL.getSearchSpace());
    }

    auto NewTuningData = std::make_unique<TemplateTuningData>(
        CD, Idx, std::move(TATuner), std::move(TAL));
    TTD = NewTuningData.get();

    TuningDataMap[InstInfo(Inst.InstKey, Inst.NTTPValues, Inst.NTTPValuesSize,
                           Inst.TypeStrings, Inst.TypeStringsCnt)] =
        std::move(NewTuningData);
  } else {
    TTD = It->second.get();
  }

  // Select a template specialization by either using an existing one or
  // instantiating a new one.
  auto &TemplateInst = TTD->selectSpecialization();

  auto FName = TemplateInst.Context.DeclName;


  if (TemplateInst.empty()) {
    // Serialization
    // TODO: This currently only works for IR tuning, not for template args
    // TODO: Accessing the filesystem every time a new instantiation is requested. For performance reasons, we should cache the available .jit.o files.
    auto Buf = MemoryBuffer::getFile(TemplateInst.Context.getMCFilename());
    if (Buf) {
      // Add to the JIT engine.
      auto Key = CD.CJ->addPrecompiledModule(std::move(Buf.get()));

      auto ImplName = TimingHelper::getImplName(FName.str());

      // Lookup the address of the generated function.
      auto SpecSymbol = CD.CJ->findSymbol(ImplName);
      assert(SpecSymbol && "Can't find the specialization just generated?");

      if (auto Err = SpecSymbol.takeError()) {
        errs() << "JIT Error: " << Err << "\n";
        fatal();
      }

      if (!SpecSymbol.getAddress())
        fatal();

      auto *FPtr = (void *) llvm::cantFail(SpecSymbol.getAddress());
      outs() << "Successfully loaded tuned function " << FName << "\n";
      return {FPtr, true};
    }
  }


  if (TemplateInst.Context.Opt->isDone()) {
    auto Best = TemplateInst.getCurrentBest();
    auto& BaseLine = TemplateInst.Instantiations[1];
    auto BaselineStats = BaseLine.updateStats();
    auto BestStats = Best->updateStats();
    JIT_INFO(outs() << "Tuning done: " << FName << "\n");
    JIT_INFO(outs() << formatv("Speedup: {0:f2}\n", BaselineStats.Mean / BestStats.Mean));
    auto* ReturnPtr = Best->FImplPtr;
    if (BaselineStats.betterThan(BestStats)) {
      ReturnPtr = BaseLine.FImplPtr;
      JIT_INFO(outs() << "Autotuner was unable to produce speedups.\n");
    }

    if (!TTD->hasTunableArgs()) {
      // TODO: Serialization only available for non-tuned template args
      outs() << "Serializing...\n";
      serialize(*Best, TemplateInst.Context.getMCFilename());
    }

    // Only enable fast lookup if template arguments are not tuned.
    return {ReturnPtr, !TTD->hasTunableArgs()};
  }

  if (TemplateInst.Context.Emitted && !TemplateInst.Instantiations.empty()) {
    auto CurrentVersion = TemplateInst.getActiveVersion();
    assert(CurrentVersion && "No active compiled version found!");

    auto Stats = CurrentVersion->updateStats();

    const unsigned MinSamples = 3;
    const double MaxRelStdErr = 0.05;

    bool Recompile = false;
    if (Stats.valid()) {
      // Recompile if the function has been called enough times and the relative
      // standard error of mean is below a certain threshold.
      Recompile =
          Stats.N >= MinSamples && Stats.getRelativeStdErr() < MaxRelStdErr;
    }

    if (!Recompile)
      return {CurrentVersion->FPtr, false};

    JIT_DEBUG(dbgs() << "Recompiling " << TemplateInst.Context.DeclName
                     << "\n");
    //JIT_REPORT({outs() << "Tuner report:\n"; printFullReport(*TTD, outs());});
    //JIT_INFO({if (clang::jit::LogLvl < LOG_REPORT) {outs() << "Tuner report:\n"; printShortReport(*TTD, outs());};});
  }

  auto Mod = llvm::CloneModule(*TemplateInst.Context.Mod);

  auto EvalRequest = TemplateInst.Context.Opt->optimize(
      Mod.get(), !TemplateInst.Context.Emitted);

  JIT_DEBUG(util::dumpModule(*Mod, "Module after optimization"));

  // Instrument optimized function for tuning
  auto SMF = Mod->getFunction(FName);

  TimingHelper TH(SMF);
  TH.createTimingWrapper();

  // Add to the JIT engine.
  MemBufferPtr MCBuffer;
  auto Key = CD.CJ->addModule(std::move(Mod), &MCBuffer);

  auto lookupFunction = [&](llvm::StringRef Name) -> void* {
    auto SpecSymbol = CD.CJ->findSymbol(Name);
    assert(SpecSymbol && "Can't find the specialization just generated?");

    if (auto Err = SpecSymbol.takeError()) {
      errs() << "JIT Error: " << Err << "\n";
      fatal();
    }

    if (!SpecSymbol.getAddress())
      fatal();

    return (void *)llvm::cantFail(SpecSymbol.getAddress());
  };

  // Lookup the address of the generated function.
  auto *FPtr = lookupFunction(FName.str());
  // Lookup the non-timed function
  auto *FImplPtr = lookupFunction(TimingHelper::getImplName(FName.str()));

  // Look up addresses of timing globals.
  auto *CJPtr = CD.CJ.get();
  auto Globals = TH.lookupGlobals([CJPtr](StringRef SymName) -> void * {
    auto Sym = CJPtr->findSymbol(SymName);
    if (auto Err = Sym.takeError()) {
      llvm::errs() << "Can't find symbol " << SymName << ": " << Err << "\n";
      return nullptr;
    }
    auto Addr = Sym.getAddress();
    if (!Addr) {
      llvm::errs() << "Unable to find address of global " << SymName << "\n";
      return nullptr;
    }
    return reinterpret_cast<void *>(Addr.get());
  });

  PtrToInstInfo.insert({FPtr, PtrData(this, InstInfo(Inst.InstKey, Inst.NTTPValues, Inst.NTTPValuesSize,
                                 Inst.TypeStrings, Inst.TypeStringsCnt))});

  TunedCodeVersion Version(TemplateInst.nextVersionID(), Key, FPtr, FImplPtr, Globals,
                           std::move(EvalRequest), std::move(MCBuffer));
  TemplateInst.add(std::move(Version));
  TemplateInst.Context.Emitted =
      true; // TODO: We probably don't need this anymore

  return {FPtr, false};
}

void* TunerDriver::finishTuning(const clang::jit::InstInfo &Inst) {
  auto It = TuningDataMap.find(Inst);
  if (It == TuningDataMap.end()) {
    llvm::errs() << "Could not find tuning data for instantiation\n";
    return nullptr;
  }
  auto& TuningData = It->second;
  auto Best = TuningData->computeOverallBest();
  if (!Best) {
    llvm::errs() << "No template instantiation found\n";
    return nullptr;
  }
  auto* BestVersion = Best->second->getCurrentBest();
  if (!BestVersion) {
    llvm::errs() << "No optimized instantiation found\n";
    return nullptr;
  }

  // Select the non-timed version of the function.
  InstData IData = {BestVersion->FImplPtr, true};
  updateActiveInstantiation(Inst, IData);
  JIT_INFO(outs() << "Finished tuning: " << Inst.Key << "\n");
  return IData.FPtr;
}

bool TunerDriver::isFinished(const clang::jit::InstInfo &Inst) {
  auto It = TuningDataMap.find(Inst);
  if (It == TuningDataMap.end()) {
    llvm::errs() << "Could not find tuning data for instantiation\n";
    return false;
  }
  auto& TuningData = It->second;
  if (TuningData->hasTunableArgs())
    return false; // TODO: Needs to be properly checked for parameter tuning
  if (TuningData->Specializations.empty())
    return false;
   auto& Opt = TuningData->Specializations[TuningData->ActiveConfig].Context.Opt;
   return Opt->isDone();
}


void* finish_tuning(void* FPtr) {
  auto It = PtrToInstInfo.find(FPtr);
  if (It != PtrToInstInfo.end()) {
    auto& PtrData = It->second;
    auto& Driver = PtrData.Driver;
    return Driver->finishTuning(PtrData.Inst);
  }
  llvm::errs() << "finish tuning: Could not find instantiation data\n";
  return nullptr;
}

bool is_finished(void* FPtr) {
  auto It = PtrToInstInfo.find(FPtr);
  if (It != PtrToInstInfo.end()) {
    auto& PtrData = It->second;
    return PtrData.Driver->isFinished(PtrData.Inst);
  }
//  llvm::errs() << "Could not find instantiation data\n";
  return true;
}

} // namespace jit
} // namespace clang
