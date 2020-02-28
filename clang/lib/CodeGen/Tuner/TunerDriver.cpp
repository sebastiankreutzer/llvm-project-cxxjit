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
    if (!Stats.Valid()) {
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
  auto Best = TemplateInst.Instantiations[BestID];
  // TODO: We assume here that the first version is the baseline, make this
  // explicit
  auto BaseLine = TemplateInst.Instantiations[1];
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
  auto BaseLine = BestSpecialization->Instantiations[1];
  OS << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                BestSpecialization->Context.DeclName, BestVersion->ID, BestStats.N,
                formatv("{0:f1}", BestStats.Mean),
                formatv("{0:p}", BestStats.getRSD()),
                formatv("{0:p}", BestStats.getRelativeStdErr()));
  OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
  OS << formatv("Speedup: ({0:p} speedup)\n", BaseLine.updateStats().Mean / BestStats.Mean - 1.0);
  OS << formatv("{0}\n", fmt_repeat("-", Header.size()));

//  auto TAKnobs = Data.TunableArgs.getKnobSet();
//  KnobState(TAKnobs, OverallBest.getValue().first).dump();
//  KnobState(BestSpecialization->Context.Opt->getKnobs(), BestVersion->Request.Cfg).dump();

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
      if (!Stats.Valid()) {
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
    auto Best = TemplateInst.Instantiations[BestID];
    // TODO: We assume here that the first version is the baseline, make this
    // explicit
    auto BaseLine = TemplateInst.Instantiations[1];
    OS << formatv("Best version: {0} ({1:p} speedup)\n", BestID,
                  BaseLine.updateStats().Mean / Best.updateStats().Mean - 1.0);
    OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
  }
  OS << formatv("{0}\n", fmt_repeat("=", Header.size()));

  if (OverallBest) {
    auto BestSpecialization = OverallBest.getValue().second;
    auto BestVersion = BestSpecialization->getCurrentBest();
    auto BestStats = BestVersion->updateStats();
    auto BaseLine = BestSpecialization->Instantiations[1];
    OS << "Overall best: \n";
    OS << formatv("{0} {1,4} {2,10}  {3,12}  {4,10}  {5,10}\n",
                  BestSpecialization->Context.DeclName, BestVersion->ID, BestStats.N,
                        formatv("{0:f1}", BestStats.Mean),
                        formatv("{0:p}", BestStats.getRSD()),
                        formatv("{0:p}", BestStats.getRelativeStdErr()));
    OS << formatv("Speedup: ({0:p} speedup)\n", BaseLine.updateStats().Mean / BestStats.Mean - 1.0);
    OS << formatv("{0}\n", fmt_repeat("-", Header.size()));
    OS << "Found configuration: \n";
    OS << "TODO\n";
//    auto TAKnobs = Data.TunableArgs.getKnobSet();
//    KnobState(TAKnobs, OverallBest.getValue().first).dump();
//    KnobState(BestSpecialization->Context.Opt->getKnobs(), BestVersion->Request.Cfg).dump();
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
    if (!Stats.Valid()) {
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
//    SmallVector<std::unique_ptr<TemplateArgKnob>, 4> TAKnobs;
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
//            TAKnobs.push_back(std::make_unique<TemplateArgKnob>(
//                Pos, Range.Min, Range.Max, Range.Min));

            return TemplateArgument(
                *CD.Ctx, SIntVal,
                CD.Ctx->getIntTypeForBitwidth(
                    32,
                    false)); // TODO: Check if expected template params matches
          }
          return {};
        });

    JIT_INFO(dbgs() << "Looking for tunable template arguments..." << "\n");
    TunableArgList TAL(BaseArgs);
    for (auto& NTTA : TunableNTTAs) {
      TAL.add(NTTA);
    }
//    for (auto &Knob : TAKnobs) {
//      JIT_INFO(dbgs() << "Template argument marked tunable: "
//                       << Knob->getArgIndex() << "\n");
//      TAL.add(std::move(Knob));
//    }

//    KnobSet TAKnobSet = TAL.getKnobSet();
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
  
  if (TemplateInst.Context.Opt->isDone()) {
    JIT_INFO(dbgs() << "Tuner has finished!\n");
    auto Best = TemplateInst.getCurrentBest();
    auto BaseLine = TemplateInst.Instantiations[1];
    outs() << "Tuning done: " << FName;
    outs() << formatv("Speedup: {0:f2}\n", BaseLine.updateStats().Mean / Best->updateStats().Mean);
    // Only enable fast lookup if template arguments are not tuned.
    return {Best->FPtr, !TTD->hasTunableArgs()};
  }

  if (TemplateInst.Context.Emitted && !TemplateInst.Instantiations.empty()) {
    auto CurrentVersion = TemplateInst.getActiveVersion();
    assert(CurrentVersion && "No active compiled version found!");

    auto Stats = CurrentVersion->updateStats();

    const unsigned MinSamples = 3;
    const double MaxRelStdErr = 0.05;

    bool Recompile = false;
    if (Stats.Valid()) {
      // Recompile if the function has been called enough times and the relative
      // standard error of mean is below a certain threshold.
      Recompile =
          Stats.N >= MinSamples && Stats.getRelativeStdErr() < MaxRelStdErr;
    }

    if (!Recompile)
      return {CurrentVersion->FPtr, false};

    JIT_DEBUG(dbgs() << "Recompiling " << TemplateInst.Context.DeclName
                     << "\n");
    JIT_REPORT({outs() << "Tuner report:\n"; printFullReport(*TTD, outs());});
    JIT_INFO({if (clang::jit::LogLvl < LOG_REPORT) {outs() << "Tuner report:\n"; printShortReport(*TTD, outs());};});
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
  auto Key = CD.CJ->addModule(std::move(Mod));

  // Lookup the address of the generated function.
  auto SpecSymbol = CD.CJ->findSymbol(FName.str());
  assert(SpecSymbol && "Can't find the specialization just generated?");

  if (auto Err = SpecSymbol.takeError()) {
    errs() << "JIT Error: " << Err << "\n";
    fatal();
  }

  if (!SpecSymbol.getAddress())
    fatal();

  auto *FPtr = (void *)llvm::cantFail(SpecSymbol.getAddress());

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

  TunedCodeVersion Version(TemplateInst.nextVersionID(), Key, FPtr, Globals,
                           std::move(EvalRequest));
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
  InstData IData = {BestVersion->FPtr, true};
  updateActiveInstantiation(Inst, IData);
  JIT_INFO(dbgs() << "Finished tuning: " << Inst.Key << "\n");
  return IData.FPtr;
}


void* finish_tuning(void* FPtr) {
  auto It = PtrToInstInfo.find(FPtr);
  if (It != PtrToInstInfo.end()) {
    auto& PtrData = It->second;
    auto& Driver = PtrData.Driver;
    return Driver->finishTuning(PtrData.Inst);
  }
  return nullptr;
}

} // namespace jit
} // namespace clang
