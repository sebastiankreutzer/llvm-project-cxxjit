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

using namespace llvm;

static const char *LOOP_NAME_TAG = "llvm.loop.id";

inline MDNode *createSelfReferencingMD(LLVMContext& Ctx, ArrayRef<Metadata*> Ops) {
  auto Dummy = MDNode::get(Ctx, {});
  SmallVector<Metadata*, 4> MDList{Dummy};
  MDList.insert(MDList.end(), Ops.begin(), Ops.end());
  auto Node = MDNode::get(Ctx, MDList);
  Node->replaceOperandWith(0, Node);
  return Node;
}

inline MDNode *getOrCreateLoopID(Loop *Loop) {
  LLVMContext &Ctx = Loop->getHeader()->getContext();
  auto LoopID = Loop->getLoopID();
  if (!LoopID) {
    LoopID = createSelfReferencingMD(Ctx, {});
    Loop->setLoopID(LoopID);
//    auto Dummy = MDNode::get(Ctx, {});
//    LoopID = MDNode::get(Ctx, {Dummy});
//    LoopID->replaceOperandWith(0, LoopID);
//    Loop->setLoopID(LoopID);
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

inline MDNode *addTag(MDNode *LoopMD, StringRef Tag) {
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

inline MDNode* addTaggedString(MDNode *LoopMD, StringRef Tag, StringRef Val) {
  auto TagMD = MDString::get(LoopMD->getContext(), Tag);
  auto ValMD = MDString::get(LoopMD->getContext(), Val);
  auto NewMD = MDNode::get(LoopMD->getContext(), {TagMD, ValMD});
  return addToLoopMD(LoopMD, NewMD);
}

inline MDNode *addTaggedBool(MDNode *LoopMD, StringRef Tag, bool Val) {
  auto C = ConstantInt::get(llvm::Type::getInt1Ty(LoopMD->getContext()), Val ? 1 : 0);
  return addTaggedConstantMD(LoopMD, Tag, C);
}

inline MDNode *addTaggedInt32(MDNode *LoopMD, StringRef Tag, int Val) {
  auto C = ConstantInt::get(llvm::Type::getInt32Ty(LoopMD->getContext()), Val);
  return addTaggedConstantMD(LoopMD, Tag, C);
}

inline MDNode *addTaggedMD(MDNode* LoopMD, StringRef Tag, MDNode* OtherMD) {
  auto TagMD = MDString::get(LoopMD->getContext(), Tag);
  auto NewMD = MDNode::get(LoopMD->getContext(), {TagMD, OtherMD});
  return addToLoopMD(LoopMD, NewMD);
}

inline MDNode *assignLoopName(Loop* Loop, StringRef Name) {
  LLVMContext &Ctx = Loop->getHeader()->getContext();
  auto TagMD = MDString::get(Ctx, LOOP_NAME_TAG);
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
    if (!TagMD || !TagMD->getString().equals(LOOP_NAME_TAG))
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

  unsigned NameAsInt = 0;
  if (Name.empty() || Name.getAsInteger(10, NameAsInt)) {
    errs() << "Loop name must be an integer\n";
    return 0;
  }
  return NameAsInt;
}

class LoopMDBuilder {
public:

  LoopMDBuilder(LLVMContext& Ctx, Loop* L = nullptr) : Ctx(Ctx), L(L) {
    if (L)
      LoopMD = getOrCreateLoopID(L);
    else
    LoopMD = createSelfReferencingMD(Ctx, {});
  }

  ~LoopMDBuilder() {
    if (L)
      L->setLoopID(LoopMD);
  }

  MDNode* getResult() {
    return LoopMD;
  }

  void append(MDNode *Node) {
    auto AddedMD = MDNode::get(Ctx, {Node});
    auto NewMD = MDNode::concatenate(LoopMD, AddedMD);
    NewMD->replaceOperandWith(0, NewMD);
    LoopMD = NewMD;
  }


  MDNode *addTaggedConstantMD(StringRef Tag, Constant *C) {
    auto StrMD = MDString::get(Ctx, Tag);
    auto ConstMD = ConstantAsMetadata::get(C);
    auto NewMD = MDNode::get(Ctx, {StrMD, ConstMD});
    append(NewMD);
    return NewMD;
  }

  MDNode* addTag(StringRef Tag) {
    auto TagMD = MDString::get(Ctx, Tag);
    auto NewMD = MDNode::get(Ctx, {TagMD});
    append(NewMD);
    return NewMD;
  }

  MDNode* addTaggedString(StringRef Tag, StringRef Val) {
    auto TagMD = MDString::get(Ctx, Tag);
    auto ValMD = MDString::get(Ctx, Val);
    auto NewMD = MDNode::get(Ctx, {TagMD, ValMD});
    append(NewMD);
    return NewMD;
  }

  MDNode *addTaggedBool(StringRef Tag, bool Val) {
    auto C = ConstantInt::get(llvm::Type::getInt1Ty(Ctx), Val ? 1 : 0);
    return addTaggedConstantMD(Tag, C);
  }

  MDNode *addTaggedInt32(StringRef Tag, int Val) {
    auto C = ConstantInt::get(llvm::Type::getInt32Ty(Ctx), Val);
    return addTaggedConstantMD(Tag, C);
  }

  MDNode* addTaggedInt32List(StringRef Tag, ArrayRef<int> Vals) {
    MDNode* NewMD = MDNode::get(Ctx, {MDString::get(Ctx, Tag)});
    for (auto i : Vals) {
      auto C = ConstantInt::get(llvm::Type::getInt32Ty(Ctx), i);
      auto ConstMD = MDNode::get(Ctx, {ConstantAsMetadata::get(C)});
      NewMD = MDNode::concatenate(NewMD, ConstMD);
    }
    append(NewMD);
    return NewMD;

  }

  inline MDNode *addTaggedMD(StringRef Tag, MDNode* OtherMD) {
    auto TagMD = MDString::get(Ctx, Tag);
    auto NewMD = MDNode::get(Ctx, {TagMD, OtherMD});
    append(NewMD);
    return NewMD;
  }

private:
  LLVMContext& Ctx;
  Loop* L;
  MDNode* LoopMD;
};


}
}

#endif // CLANG_LOOPMD_H
