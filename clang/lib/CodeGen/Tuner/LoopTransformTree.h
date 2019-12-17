//
// Created by sebastian on 16.12.19.
//

#ifndef LLVM_LOOPTRANSFORMTREE_H
#define LLVM_LOOPTRANSFORMTREE_H

#include "LoopMD.h"

#include "llvm/ADT/SmallVector.h"

using namespace llvm;

namespace clang {
namespace  jit {

namespace MDTags {
static const char * LOOP_ID_TAG = "llvm.loop.id";
static const char *TILE_ENABLE_TAG = "llvm.tile.enable";
static const char *TILE_DEPTH_TAG = "llvm.tile.depth";
static const char *TILE_SIZE_TAG = "llvm.tile.size";
static const char *TILE_PEEL_TAG = "llvm.tile.peel";
static const char *TILE_FOLLOWUP_FLOOR_TAG = "llvm.tile.followup_floor";
static const char *TILE_FOLLOWUP_TILE_TAG = "llvm.tile.followup_tile";
}

class LoopNode;

using LoopNodePtr = std::unique_ptr<LoopNode>;


// This is a placeholder class for the actual metadata objects.
// We can't use regular MDNodes because we need to replicate these across multiple reoptimized modules.
template<typename T>
struct MDAttr {
  MDAttr(std::string Name, T Val)
    : Name(std::move(Name)), Val(std::move(Val)) {
  }

  std::string Name;
  T Val;
};

using IntAttr = MDAttr<int>;
using BoolAttr = MDAttr<bool>;
using StringAttr = MDAttr<SmallString<8>>;
using RedirectAttr = MDAttr<LoopNode*>;

class LoopTransformTree {
public:

  LoopTransformTree() {
  }

  LoopTransformTree(const LoopTransformTree&) = delete;
  LoopTransformTree& operator=(const LoopTransformTree&) = delete;

  LoopNode* makeVirtualNode();

  LoopNode* makeNode(std::string Name = "");

  LoopNode* getNode(StringRef Name) {
    auto It = Nodes.find(Name);
    return It == Nodes.end() ? nullptr : It->second.get();
  }

  LoopNode* getRoot() {
    return Root;
  }

  void setRoot(LoopNode* Node) {
    this->Root = Node;
  }

private:
  LoopNode* Root{nullptr};
  StringMap<LoopNodePtr> Nodes;
  unsigned NodeCount{0};
};

class LoopNode {
  LoopNode(LoopTransformTree* Tree, bool Virtual, StringRef LoopName, LoopNode* Parent=nullptr)
      : Tree(Tree), IsVirtualLoop(Virtual), LoopName(LoopName), Parent(Parent)
  {
  }
  friend class LoopTransformTree;
public:

  using LoopList = SmallVector<LoopNode*, 2>;

  struct AttributeBag {
    SmallVector<IntAttr, 4> IntAttrs;
    SmallVector<BoolAttr, 4> BoolAttrs;
    SmallVector<StringAttr, 2> StringAttrs;
    SmallVector<RedirectAttr, 2> RedirectAttrs;
  };

  LoopTransformTree* getTransformTree() {
    return Tree;
  }

  void addSubLoop(LoopNode* LN) {
    this->SubLoops.push_back(LN);
    LN->Parent = this;
  }

  /// Adds a successor node that is the result of a transformation.
  /// \param LN
  void addSuccesor(LoopNode* LN) {
    // TODO: Add inherit flag for attributes
    this->Successor = LN;
    LN->Predecessor = this;
  }

  LoopNode* getOriginalLoop() {
    if (Predecessor)
      return Predecessor->getOriginalLoop();
    return this;
  }

  LoopNode* getSuccessor() {
    return Successor;
  }

  LoopNode* getLastSuccessor() {
    if (Successor)
      return Successor->getLastSuccessor();
    return this;
  }

  LoopNode* getParent() {
    return Parent;
  }

  /// If this loop is virtual and has no parent, return the parent of the last predecessor that has one.
  /// \return
  LoopNode* getEffectiveParent() {
    if (Parent)
      return Parent;
    if (Predecessor)
      return Predecessor->getEffectiveParent();
    return nullptr;
  }

  StringRef getLoopName() {
    return LoopName;
  }

  bool hasSubLoop() {
    return !SubLoops.empty();
  }

  LoopNode* getFirstSubLoop() {
    return SubLoops.empty() ? nullptr : SubLoops.front();
  }

  llvm::iterator_range<LoopList::iterator> subLoops() {
    return make_range(SubLoops.begin(), SubLoops.end());
  }

  llvm::iterator_range<LoopList::const_iterator> subLoops() const {
    return make_range(SubLoops.begin(), SubLoops.end());
  }

  unsigned getDepth() const {
    if (!Parent)
      return 1;
    return Parent->getDepth() + 1;
  }

  unsigned getMaxDepth() const {
    unsigned MaxDepth = getDepth();
    for (auto& SL : SubLoops) {
      MaxDepth = std::max(MaxDepth, SL->getMaxDepth());
    }
    return MaxDepth;
  }

  unsigned getRelativeMaxDepth() const {
    return getMaxDepth() - getDepth() + 1;
  }

  bool isPerfectlyNested() const {
    if (SubLoops.empty())
      return true;
    if (SubLoops.size() > 1)
      return false;
    return SubLoops.front()->isPerfectlyNested();
  }

  /// Virtual loops are those created by transformations.
  /// Metadata for these loops must be created when the attributes are added.
  /// \return
  bool isVirtual() const {
    return IsVirtualLoop;
  }

  void addIntAttribute(StringRef Name, int Val) {
    Attrs.IntAttrs.emplace_back(Name, Val);
  }

  void addBoolAttribute(StringRef Name, bool Val) {
    Attrs.BoolAttrs.emplace_back(Name, Val);
  }

  void addStringAttribute(StringRef Name, StringRef Val) {
    Attrs.StringAttrs.emplace_back(Name, Val);
  }

  void addRedirectAttribute(StringRef Name, LoopNode* Val) {
    Attrs.RedirectAttrs.emplace_back(Name, Val);
  }

  AttributeBag& getAttributes() {
    return Attrs;
  }

private:
  LoopTransformTree* Tree;
  bool IsVirtualLoop;
  SmallString<8> LoopName;
  LoopNode* Parent;
  LoopList SubLoops;
  LoopNode* Predecessor{nullptr};
  LoopNode* Successor{nullptr};
  AttributeBag Attrs;
};

void applyTiling(LoopNode *RootNode);


}
}



#endif //LLVM_LOOPTRANSFORMTREE_H
