//
// Created by sebastian on 02.10.19.
//

#ifndef CLANG_LOOPMD_H
#define CLANG_LOOPMD_H

#include "llvm/Analysis/LoopInfo.h"
#include "llvm/IR/Constants.h"
#include "llvm/IR/Metadata.h"

namespace clang {
namespace jit {

static const char *NameTag = "loop.name";

inline MDNode *getOrCreateLoopID(Loop *Loop) {
  LLVMContext &Ctx = Loop->getHeader()->getContext();
  auto LoopID = Loop->getLoopID();
  if (!LoopID) {
    auto Dummy = MDNode::get(Ctx, {});
    LoopID = MDNode::get(Ctx, {Dummy});
    LoopID->replaceOperandWith(0, LoopID);
    Loop->setLoopID(LoopID);
  }
  return LoopID;
}

inline MDNode *addToLoopMD(MDNode *LoopMD, MDNode *Node) {
  LLVMContext &Ctx = LoopMD->getContext();
  auto AddedMD = MDNode::get(Ctx, {Node});
  auto NewMD = MDNode::concatenate(LoopMD, AddedMD);
  NewMD->replaceOperandWith(0, NewMD);
  return NewMD;
}

inline MDNode *addTagMD(MDNode *LoopMD, StringRef Tag) {
  auto StrMD = MDString::get(LoopMD->getContext(), Tag);
  auto TagMD = MDNode::get(LoopMD->getContext(), {StrMD});
  return addToLoopMD(LoopMD, TagMD);
}

inline MDNode *addTaggedConstantMD(MDNode *LoopMD, StringRef Tag, Constant *C) {
  auto StrMD = MDString::get(LoopMD->getContext(), Tag);
  auto ConstMD = ConstantAsMetadata::get(C);
  auto NewMD = MDNode::get(LoopMD->getContext(), {StrMD, ConstMD});
  return addToLoopMD(LoopMD, NewMD);
}

inline MDNode *addTaggedBool(MDNode *LoopMD, StringRef Tag, bool Val) {
  auto C = ConstantInt::get(Type::getInt1Ty(LoopMD->getContext()), Val ? 1 : 0);
  return addTaggedConstantMD(LoopMD, Tag, C);
}

inline MDNode *addTaggedInt32(MDNode *LoopMD, StringRef Tag, int Val) {
  auto C = ConstantInt::get(Type::getInt32Ty(LoopMD->getContext()), Val);
  return addTaggedConstantMD(LoopMD, Tag, C);
}

inline MDNode *assignLoopName(Loop* Loop, StringRef Name) {
  LLVMContext &Ctx = Loop->getHeader()->getContext();
  auto TagMD = MDString::get(Ctx, NameTag);
  auto NameStr = MDString::get(Ctx, Name);
  auto NameMD = MDNode::get(Ctx, {TagMD, NameStr});
  auto LoopID = addToLoopMD(getOrCreateLoopID(Loop), NameMD);
  Loop->setLoopID(LoopID);
  return NameMD;
}

inline MDNode *assignLoopName(Loop *Loop, unsigned Name) {
  return assignLoopName(Loop, std::to_string(Name));
}

inline StringRef getLoopName(Loop* Loop) {
  auto LoopID = Loop->getLoopID();
  if (!LoopID || LoopID->getNumOperands() < 2) {
    return "";
  }
  for (auto &Op : LoopID->operands()) {
    auto MD = dyn_cast<MDNode>(Op);
    if (!MD || MD->getNumOperands() != 2)
      continue;
    auto TagMD = dyn_cast<MDString>(MD->getOperand(0));
    auto NameMD = dyn_cast<MDString>(MD->getOperand(1));
    if (!TagMD || !TagMD->getString().equals(NameTag))
      continue;
    if (!NameMD) {
      errs() << "Malformed loop name metadata\n";
      break;
    }
    return NameMD->getString();
  }
  return "";
}

inline unsigned getLoopNameAsInt(Loop *Loop) {

  StringRef Name = getLoopName(Loop);

  unsigned NameAsInt = InvalidKnobID;
  if (Name.empty() || Name.getAsInteger(10, NameAsInt)) {
    errs() << "Loop name must be an integer\n";
    return InvalidKnobID;
  }
  return NameAsInt;
}


}
}

#endif // CLANG_LOOPMD_H
