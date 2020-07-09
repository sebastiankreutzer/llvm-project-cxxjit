//
// Created by sebastian on 07.10.19.
//

#ifndef CLANG_TIMINGHELPER_H
#define CLANG_TIMINGHELPER_H

#include "llvm/IR/IRBuilder.h"
#include "llvm/Support/raw_ostream.h"
#include <string>

// namespace llvm {
// class Function;
// class GlobalVariable;
// class Value;
//}

namespace clang {
namespace jit {

struct TimingGlobals {
  double *VarN{nullptr};
  int64_t *Cycles{nullptr};
  int64_t *CallCount{nullptr};
  double *MeanCycles{nullptr};

  bool Valid() { return VarN && Cycles && CallCount && MeanCycles; }
};

class TimingHelper {

  llvm::Function *TimedFunction;

  std::string TotalCyclesGlobalName;
  std::string MeanCyclesGlobalName;
  std::string CallCountGlobalName;
  std::string VarNGlobalName;

  llvm::Value *insertRDTSCP(llvm::IRBuilder<> &IRB);

  llvm::Value *instrumentPreCall(llvm::IRBuilder<> &IRB);

  void instrumentPostCall(llvm::IRBuilder<> &IRB,
                          llvm::GlobalVariable *CallCount,
                          llvm::GlobalVariable *CycleCount,
                          llvm::GlobalVariable *MeanCycles,
                          llvm::GlobalVariable *VarN, llvm::Value *StartCycles);

  template<typename T, typename F>
  T *fetchGlobal(F &&SymLookupFn, llvm::StringRef Name) {
    auto Addr = SymLookupFn(Name);
    return reinterpret_cast<T *>(Addr);
  }

public:
  explicit TimingHelper(llvm::Function *TimedFunction)
      : TimedFunction(TimedFunction) {}

  static std::string getImplName(std::string OriginalName) {
    return "__clangjit_impl_" + OriginalName;
  }

  template<typename F>
  TimingGlobals lookupGlobals(F &&SymLookupFn) {
    TimingGlobals TG;
    TG.CallCount =
        fetchGlobal<int64_t>(std::forward<F>(SymLookupFn), CallCountGlobalName);
    TG.Cycles = fetchGlobal<int64_t>(std::forward<F>(SymLookupFn),
                                     TotalCyclesGlobalName);
    TG.MeanCycles =
        fetchGlobal<double>(std::forward<F>(SymLookupFn), MeanCyclesGlobalName);
    TG.VarN = fetchGlobal<double>(std::forward<F>(SymLookupFn), VarNGlobalName);
    return TG;
  }

  llvm::Function *createTimingWrapper();
};

}
}

#endif // CLANG_TIMINGHELPER_H
