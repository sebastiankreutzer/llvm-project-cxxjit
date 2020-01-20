//
// Created by sebastian on 07.10.19.
//

#include "TimingHelper.h"

#include "llvm/IR/Function.h"
#include "llvm/IR/Value.h"
#include <sstream>

using namespace llvm;

namespace clang {
namespace jit {

Function *TimingHelper::createTimingWrapper() {
  std::string FName = TimedFunction->getName().str();
  auto *M = TimedFunction->getParent();

  auto &C = M->getContext();

  auto make_global = [&](llvm::Type *GType, Constant *Init,
                         const char *Suffix) -> llvm::GlobalVariable * {
    const char *Prefix = "__clangjit_";
    std::stringstream ss;
    ss << Prefix << FName << "_"
       << Suffix; // TODO: Probably not the fastest way to do this
    auto Name = ss.str();
    M->getOrInsertGlobal(Name, GType);
    auto *Global = M->getGlobalVariable(Name);
    Global->setInitializer(Init);
    Global->setLinkage(
        GlobalVariable::InternalLinkage); // TODO: Correct linkage type?
    return Global;
  };

  auto make_global_int64 = [&](const char *Suffix) -> llvm::GlobalVariable * {
    auto *GType = llvm::Type::getInt64Ty(C);
    return make_global(GType, ConstantInt::get(GType, 0, false), Suffix);
  };

  auto make_global_double = [&](const char *Suffix) -> llvm::GlobalVariable * {
    auto *GType = llvm::Type::getDoubleTy(C);
    return make_global(GType, ConstantFP::get(GType, 0.0), Suffix);
  };

  // NOTE: We must save the names here as the module might be modified after
  // having been added to the JIT engine
  auto TotalCyclesGlobal = make_global_int64("cycles");
  TotalCyclesGlobalName = TotalCyclesGlobal->getName();
  auto MeanCyclesGlobal = make_global_double("mean_cycles");
  MeanCyclesGlobalName = MeanCyclesGlobal->getName();
  auto CallCountGlobal = make_global_int64("count");
  CallCountGlobalName = CallCountGlobal->getName();
  auto VarNGlobal = make_global_double("var_n");
  VarNGlobalName = VarNGlobal->getName();

  std::string FImplName = getImplName(FName);
  TimedFunction->setName(FImplName);

  Function *Wrapper = Function::Create(TimedFunction->getFunctionType(),
                                       Function::ExternalLinkage, FName, M);
  // Wrapper->stealArgumentListFrom(*F);

  auto *EB = BasicBlock::Create(C, "", Wrapper);
  IRBuilder<> IRB(EB);

  Value *CyclesStart = instrumentPreCall(IRB);

  // TODO: Not sure what's the best way to get the argument list here
  llvm::SmallVector<Value *, 8> ArgList;
  for (auto &&arg : Wrapper->args()) {
    ArgList.emplace_back(&arg);
  }

  auto *FCall = IRB.CreateCall(TimedFunction, ArgList);

  instrumentPostCall(IRB, CallCountGlobal, TotalCyclesGlobal, MeanCyclesGlobal,
                     VarNGlobal, CyclesStart);

  if (TimedFunction->getReturnType()->isVoidTy()) {
    IRB.CreateRetVoid();
  } else {
    IRB.CreateRet(FCall);
  }

  return Wrapper;
}

Value *TimingHelper::insertRDTSCP(IRBuilder<> &IRB) {
  llvm::Module *M = IRB.GetInsertBlock()->getModule();
  // TODO: Limited to x86 for now. Replace with llvm.readcyclecounter intrinsic?
  auto *RDTSCP = Intrinsic::getDeclaration(M, Intrinsic::x86_rdtscp);
  auto *Call = IRB.CreateCall(RDTSCP);
  return IRB.CreateExtractValue(Call, {0}); // 64 bit value
}

Value *TimingHelper::instrumentPreCall(IRBuilder<> &IRB) {
  return insertRDTSCP(IRB);
}

void TimingHelper::instrumentPostCall(
    IRBuilder<> &IRB, GlobalVariable *CallCount, GlobalVariable *CycleCount,
    GlobalVariable *MeanCycles, GlobalVariable *VarN, Value *StartCycles) {

  auto *StopCycles = insertRDTSCP(IRB);

  // Update cycles
  // RDTSCP should be 64bit, so we can probably ignore overflows
  auto *CurrentTotal = IRB.CreateLoad(CycleCount);
  auto *Elapsed = IRB.CreateSub(StopCycles, StartCycles);
  auto *ElapsedFloat = IRB.CreateSIToFP(Elapsed, IRB.getDoubleTy());
  auto *NewTotal = IRB.CreateAdd(CurrentTotal, Elapsed);
  IRB.CreateStore(NewTotal, CycleCount);

  // Update call count
  auto *OldCount = IRB.CreateLoad(CallCount);
  auto *Incd = IRB.CreateAdd(OldCount, IRB.getInt64(1));
  IRB.CreateStore(Incd, CallCount);

  auto *F = IRB.GetInsertBlock()->getParent();

  auto *IfBB = BasicBlock::Create(IRB.getContext(), "not_called_before", F);
  auto *ElseBB = BasicBlock::Create(IRB.getContext(), "called_before", F);
  auto *RetBB = BasicBlock::Create(IRB.getContext(), "return", F);

  // TODO: Remove this eventually, for debugging only
  // IRB.CreateCall(ReportFn, {Elapsed});

  // Branch
  IRB.CreateCondBr(IRB.CreateICmpEQ(OldCount, IRB.getInt64(0)), IfBB, ElseBB);

  // Call count is zero
  IRB.SetInsertPoint(IfBB);
  IRB.CreateStore(ElapsedFloat, MeanCycles);
  IRB.CreateBr(RetBB);

  // Call count is not zero
  IRB.SetInsertPoint(ElseBB);
  // Update mean
  auto *OldMean = IRB.CreateLoad(MeanCycles);
  auto *T1 = IRB.CreateFSub(ElapsedFloat, OldMean);
  auto *NewMean = IRB.CreateFAdd(
      OldMean, IRB.CreateFDiv(T1, IRB.CreateSIToFP(Incd, IRB.getDoubleTy())));
  IRB.CreateStore(NewMean, MeanCycles);

  // Update variance
  auto *OldVarN = IRB.CreateLoad(VarN);
  auto *VarNInc = IRB.CreateFMul(T1, IRB.CreateFSub(ElapsedFloat, NewMean));
  auto *NewVarN = IRB.CreateFAdd(OldVarN, VarNInc);
  IRB.CreateStore(NewVarN, VarN);
  IRB.CreateBr(RetBB);

  IRB.SetInsertPoint(RetBB);
}

}
}