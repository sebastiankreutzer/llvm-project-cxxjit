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

namespace util {

template <class MapIterator>
class MapValueIterator : public MapIterator {
public:
  using value_type = decltype(MapIterator::value_type::second);
  using pointer    = value_type*;
  using reference  = value_type&;

  MapValueIterator(const MapIterator& other) : MapIterator(other) {
  }

  reference operator*() const {
    return this->MapIterator::operator*().second;
  }

//  pointer operator->() const {
//    return *(this->MapIterator::operator*().second);
//  }

  reference operator[](size_t n) const {
    return this->MapIterator::operator[](n).second;
  }
};

template <typename Iterator>
MapValueIterator<Iterator> map_value_iterator(Iterator t) {
  return MapValueIterator<Iterator>(t);
}

// Based on https://jonasdevlieghere.com/containers-of-unique-pointers/
// Allows to create iterators for containers of std::unique_ptr
template <class BaseIterator>
class DereferenceIterator : public BaseIterator {
public:
  using value_type = typename BaseIterator::value_type::element_type;
  using pointer    = value_type*;
  using reference  = value_type&;

  DereferenceIterator(const BaseIterator& other) : BaseIterator(other) {
  }

  pointer operator*() const {
    return this->BaseIterator::operator*().get();
  }

  pointer operator->() const {
    return this->BaseIterator::operator*().get();
  }

  pointer operator[](size_t n) const {
    return this->BaseIterator::operator[](n).get();
  }
};

template <typename Iterator>
DereferenceIterator<Iterator> dereference_iterator(Iterator t) {
  return DereferenceIterator<Iterator>(t);
}

}  // namespace util

namespace MDTags {
//extern const char *LOOP_ID_TAG;
extern const char* DISABLE_NONFORCED;
extern const char *TILE_ENABLE_TAG;
extern const char *TILE_DEPTH_TAG;
extern const char *TILE_SIZE_TAG;
extern const char *TILE_PEEL_TAG;
extern const char *TILE_FOLLOWUP_FLOOR_TAG;
extern const char *TILE_FOLLOWUP_TILE_TAG;
extern const char *INTERCHANGE_ENABLE_TAG;
extern const char *INTERCHANGE_DEPTH_TAG;
extern const char *INTERCHANGE_PERMUTATION_TAG;
extern const char *INTERCHANGE_FOLLOWUP_TAG;
extern const char *UNROLL_AND_JAM_ENABLE_TAG;
extern const char *UNROLL_AND_JAM_COUNT_TAG;
extern const char *UNROLL_AND_JAME_FOLLOWUP_UNROLLED_TAG;
extern const char *UNROLL_ENABLE_TAG;
extern const char *UNROLL_COUNT_TAG;
extern const char *UNROLL_FULL_TAG;

}

class LoopNode;

using LoopNodePtr = std::unique_ptr<LoopNode>;


// This is a placeholder class for the actual metadata objects.
// We can't use regular MDNodes because we need to replicate these across multiple reoptimized modules.
template<typename T>
struct MDAttr {
  MDAttr(std::string Name, T Val, bool Inherit = false)
    : Name(std::move(Name)), Val(std::move(Val)), Inherit(Inherit){
  }

  std::string Name;
  T Val;
  bool Inherit;
};

using IntAttr = MDAttr<int>;
using IntListAttr = MDAttr<SmallVector<int, 4>>;
using BoolAttr = MDAttr<bool>;
using StringAttr = MDAttr<SmallString<8>>;
using FollowupAttr = MDAttr<LoopNode*>;
using TagAttr = MDAttr<std::nullptr_t>; // TODO: this is dumb




class LoopTransformTree {
public:

  using node_iterator = util::MapValueIterator<StringMap<LoopNodePtr>::iterator>;
  using const_node_iterator = util::MapValueIterator<StringMap<LoopNodePtr>::const_iterator>;

  LoopTransformTree() {
  }

  LoopTransformTree(const LoopTransformTree&) = delete;
  LoopTransformTree& operator=(const LoopTransformTree&) = delete;

  LoopTransformTree(LoopTransformTree&& Rhs) noexcept;
  LoopTransformTree& operator=(LoopTransformTree&& Rhs) noexcept {
    if (this != &Rhs) {
      *this = std::move(Rhs);
    }
    return *this;
  }

  LoopNode* makeVirtualNode();

  LoopNode* makeNode(std::string Name = "");

  LoopNode* getNode(StringRef Name) const {
    auto It = Nodes.find(Name);
    return It == Nodes.end() ? nullptr : It->second.get();
  }

  LoopNode* getRoot() const {
    return Root;
  }

  void setRoot(LoopNode* Node);

  size_t size() const {
    return Nodes.size();
  }

  node_iterator nodes_begin() {
    return util::map_value_iterator(Nodes.begin());
  }

  node_iterator nodes_end() {
    return util::map_value_iterator(Nodes.end());
  }

  iterator_range<node_iterator> nodes() {
    return make_range(nodes_begin(), nodes_end());
  }

  const_node_iterator nodes_begin() const {
    return util::map_value_iterator(Nodes.begin());
  }

  const_node_iterator nodes_end() const {
    return util::map_value_iterator(Nodes.end());
  }

  iterator_range<const_node_iterator> nodes() const {
    return make_range(nodes_begin(), nodes_end());
  }

  std::unique_ptr<LoopTransformTree> clone() const;

private:
  LoopNode* cloneNode(LoopNode* Node);

private:
  LoopNode* Root{nullptr};
  StringMap<LoopNodePtr> Nodes;
  unsigned NodeCount{0};
};

using LoopTransformTreePtr = std::unique_ptr<LoopTransformTree>;

class LoopNode {

  LoopNode(LoopTransformTree* Tree, bool Virtual, StringRef LoopName, LoopNode* Parent=nullptr)
      : Tree(Tree), IsVirtualLoop(Virtual), LoopName(LoopName), Parent(Parent)
  {
  }

  LoopNode(const LoopNode&) = delete;
  LoopNode& operator=(const LoopNode&) = delete;

  friend class LoopTransformTree;
public:

  enum TransformFlags {
    TILED_TILE = 1,
    TILED_FLOOR = 1 << 1,
    INTERCHANGED = 1 << 2,
    UNROLLED = 1 << 3,
    JAMMED = 1 << 4
  };

  using LoopList = SmallVector<LoopNode*, 2>;

  struct TripCountInfo {
    unsigned TripCount;
    bool IsExact;

    TripCountInfo(unsigned TripCount, bool IsExact) : TripCount(TripCount), IsExact(IsExact) {}

    TripCountInfo() : TripCountInfo(0, false){}

    bool hasInfo() {
      return TripCount != 0;
    }

  };

  struct AttributeBag {
    SmallVector<TagAttr, 1> Tags;
    SmallVector<IntAttr, 4> IntAttrs;
    SmallVector<BoolAttr, 4> BoolAttrs;
    SmallVector<StringAttr, 2> StringAttrs;
    SmallVector<IntListAttr, 1> IntListAttrs;
    SmallVector<FollowupAttr, 2> FollowupAttrs;
  };

  TripCountInfo& getTripCountInfo() {
    return TCI;
  }

  const TripCountInfo& getTripCountInfo() const {
    return TCI;
  }

  LoopTransformTree* getTransformTree() {
    return Tree;
  }

  unsigned getFlags() const {
    return Flags;
  }

  void setFlag(unsigned Flag) {
    Flags |= Flag;
  }

  void clearFlags() {
    Flags = 0;
  }

  bool isSet(unsigned Flag) {
    return (bool) (Flags & Flag);
  }

  bool isSetInPredecessors(unsigned Flag) {
    return isSet(Flag) || (Predecessor ? Predecessor->isSetInPredecessors(Flag) : false);
  }

  void addSubLoop(LoopNode* LN) {
    this->SubLoops.push_back(LN);
    LN->Parent = this;
  }

  /// Adds a successor node that is the result of a transformation.
  /// \param LN
  void addSuccesor(LoopNode* LN) {
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

  const LoopNode* getSuccessor() const {
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
  LoopNode* getEffectiveParent() const {
    if (Parent)
      return Parent;
    if (Predecessor)
      return Predecessor->getEffectiveParent();
    return nullptr;
  }

  StringRef getLoopName() const {
    return LoopName;
  }

  bool hasSubLoop() const {
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
    auto P = getEffectiveParent();
    if (!P)
      return 1;
    return P->getDepth() + 1;
  }

  unsigned getMaxDepth() const {
    unsigned MaxDepth = getDepth();
    for (auto& SL : SubLoops) {
      MaxDepth = std::max(MaxDepth, SL->getLastSuccessor()->getMaxDepth());
    }
    return MaxDepth;
  }

  unsigned getRelativeMaxDepth() const {
    return getMaxDepth() - getDepth() + 1;
  }

  bool isTightlyNested() const {
    if (SubLoops.empty())
      return true;
    if (SubLoops.size() > 1)
      return false;
    return SubLoops.front()->getLastSuccessor()->isTightlyNested();
  }

  /// Virtual loops are those created by transformations.
  /// Metadata for these loops must be created when the attributes are added.
  /// \return
  bool isVirtual() const {
    return IsVirtualLoop;
  }

  void addTagAttribute(StringRef Name, bool Inherit=false) {
    Attrs.Tags.emplace_back(Name, nullptr, Inherit);
  }

  void addIntAttribute(StringRef Name, int Val, bool Inherit=false) {
    Attrs.IntAttrs.emplace_back(Name, Val, Inherit);
  }

  void addIntListAttribute(StringRef Name, ArrayRef<int> Val, bool Inherit=false) {
    SmallVector<int, 4> Perm;
    Perm.insert(Perm.begin(), Val.begin(), Val.end());
    Attrs.IntListAttrs.emplace_back(Name, std::move(Perm), Inherit);
  }

  void addBoolAttribute(StringRef Name, bool Val, bool Inherit=false) {
    Attrs.BoolAttrs.emplace_back(Name, Val, Inherit);
  }

  void addStringAttribute(StringRef Name, StringRef Val, bool Inherit=false) {
    Attrs.StringAttrs.emplace_back(Name, Val, Inherit);
  }

  void addRedirectAttribute(StringRef Name, LoopNode* Val, bool Inherit=false) {
    Attrs.FollowupAttrs.emplace_back(Name, Val, Inherit);
  }

  AttributeBag& getAttributes() {
    return Attrs;
  }

  bool hasInterchangeBarrier() {
    return InterchangeBarrier;
  }

  void setInterchangeBarrier() {
    InterchangeBarrier = true;
  }

private:
  LoopTransformTree* Tree;
  bool IsVirtualLoop;
  SmallString<8> LoopName;
  LoopNode* Parent;
  LoopList SubLoops;
  LoopNode* Predecessor{nullptr};
  LoopNode* Successor{nullptr};
  unsigned Flags{0};
  AttributeBag Attrs;
  TripCountInfo TCI;
  bool InterchangeBarrier{false};
};




}
}



#endif //LLVM_LOOPTRANSFORMTREE_H
