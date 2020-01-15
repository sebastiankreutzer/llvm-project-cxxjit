//
// Created by sebastian on 27.09.19.
//

#include "LoopTransformTree.h"
#include "Debug.h"
#include "LoopMD.h"
#include "llvm/Analysis/LoopPass.h"

using namespace llvm;

namespace clang {
namespace jit {

class LoopTransformTreeApplicator : public llvm::LoopPass {

private:
  SmallVector<LoopTransformTree*, 2> LoopTrees;


  MDNode* makeLoopMD(LLVMContext& Ctx, LoopNode* Node, SmallVector<MDNode*, 4>* Inherited = nullptr, Loop* L = nullptr) {
    LoopMDBuilder Builder(Ctx, L);

//    MDNode* LoopMD = nullptr;
//    if (Node->isVirtual()) {
//      assert(!L && "Node is marked virtual but a corresponding loop is given");
//      LoopMD = createSelfReferencingMD(Ctx, {});
//    } else {
//      assert(L && "Non-virtual nodes must correspond to an existing loop");
//      LoopMD = L->getLoopID();
//    }
    // TODO: This is ugly... not sure how to improve, maybe use unions?
    SmallVector<MDNode*, 4> Inheritable;

    auto saveInheritable = [&Inheritable](MDNode* MD, bool Inherit) {
      if (Inherit) {
        Inheritable.push_back(MD);
      }
    };

    if (Inherited) {
      for (auto& MD: *Inherited) {
        Builder.append(MD);
        // TODO: Should this be non-transitive?
        Inheritable.push_back(MD);
      }
    }

    auto& Attributes = Node->getAttributes();
    for (auto& IntAttr : Attributes.IntAttrs) {
//      LoopMD = addTaggedInt32(LoopMD, IntAttr.Name, IntAttr.Val);
        saveInheritable(Builder.addTaggedInt32(IntAttr.Name, IntAttr.Val), IntAttr.Inherit);
    }
    for (auto& IntListAttr : Attributes.IntListAttrs) {
      saveInheritable(Builder.addTaggedInt32List(IntListAttr.Name, IntListAttr.Val), IntListAttr.Inherit);
    }
    for (auto& StringAttr : Attributes.StringAttrs) {
//      LoopMD = addTaggedString(LoopMD, StringAttr.Name, StringAttr.Val);
        saveInheritable(Builder.addTaggedString(StringAttr.Name, StringAttr.Val), StringAttr.Inherit);
    }
    for (auto& BoolAttr : Attributes.BoolAttrs) {
//      LoopMD = addTaggedBool(LoopMD, BoolAttr.Name, BoolAttr.Val);
      saveInheritable(Builder.addTaggedBool(BoolAttr.Name, BoolAttr.Val), BoolAttr.Inherit);
    }
    for (auto& Tag : Attributes.Tags) {
      saveInheritable(Builder.addTag(Tag.Name), Tag.Inherit);
    }
    // Recursively resolve followups
    for (auto& FollowupAttr : Attributes.FollowupAttrs) {
      auto MD = makeLoopMD(Ctx, FollowupAttr.Val, &Inheritable);
      Builder.addTaggedMD(FollowupAttr.Name, MD);
//      LoopMD = addTaggedMD(LoopMD, FollowupAttr.Name, MD);
    }
//    if (L)
//      L->setLoopID(LoopMD);
    return Builder.getResult();
  }


  bool processLoopTree(const LoopTransformTree &Tree, Loop *L) {
    auto Name = getLoopName(L);
    if (Name.empty())
      return false;
    auto Node = Tree.getNode(Name);
    if (!Node)
      return false;
    assert(!Node->isVirtual() && "Actual loops cannot be virtual - there is a bug in the tree creation pass");
    assert(Node->getOriginalLoop() == Node && "Matched loop is not the original one!");

    auto& Ctx = L->getHeader()->getContext();
    makeLoopMD(Ctx, Node, nullptr, L);
    return true;
  }

public:
  static char ID;

  explicit LoopTransformTreeApplicator() : LoopPass(ID) {};

  void setTransformTrees(ArrayRef<LoopTransformTree*> LoopTrees) {
    this->LoopTrees.clear();
    this->LoopTrees.insert(this->LoopTrees.begin(), LoopTrees.begin(), LoopTrees.end());
  }

  bool runOnLoop(Loop *Loop, LPPassManager &LPM) override {

    if (Loop->getHeader()->getParent()->isDeclarationForLinker()) {
      // We don't want to consider loops in functions that are marked available_externally
      return false;
    }

    for (auto& Tree : LoopTrees) {
      if (processLoopTree(*Tree, Loop))
        return true;
    }

    return false;

  }

}; // end class

char LoopTransformTreeApplicator::ID = 0;
static RegisterPass<LoopTransformTreeApplicator> Register("loop-tree-applicator",
                                              "Make loop transformation tree",
                                              false /* only looks at CFG*/,
                                              false /* analysis pass */);

llvm::Pass *createLoopTransformTreeApplicatorPass(ArrayRef<LoopTransformTree*> LoopTrees) {
  auto LTC = new LoopTransformTreeApplicator();
  LTC->setTransformTrees(LoopTrees);
  return LTC;
}

}
}