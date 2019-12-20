//
// Created by sebastian on 16.12.19.
//

#include "LoopTransformTree.h"



namespace clang {
namespace jit {

namespace MDTags {
const char *LOOP_ID_TAG = "llvm.loop.id";
const char *DISABLE_NONFORCED = "llvm.loop.disable_nonforced";
const char *TILE_ENABLE_TAG = "llvm.loop.tile.enable";
const char *TILE_DEPTH_TAG = "llvm.loop.tile.depth";
const char *TILE_SIZE_TAG = "llvm.loop.tile.size";
const char *TILE_PEEL_TAG = "llvm.loop.tile.peel";
const char *TILE_FOLLOWUP_FLOOR_TAG = "llvm.loop.tile.followup_floor";
const char *TILE_FOLLOWUP_TILE_TAG = "llvm.loop.tile.followup_tile";
const char *INTERCHANGE_ENABLE_TAG = "llvm.loop.interchange.enable";
const char *INTERCHANGE_DEPTH_TAG = "llvm.loop.interchange.depth";
const char *INTERCHANGE_PERMUTATION_TAG = "llvm.loop.interchange.permutation";
const char *INTERCHAGNE_FOLLOWUP_TAG = "llvm.loop.interchange.followup_interchanged";
}

LoopTransformTree::LoopTransformTree(LoopTransformTree &&Rhs) noexcept
    : Root(Rhs.Root),Nodes(std::move(Rhs.Nodes)), NodeCount(Rhs.NodeCount) {
  // Update tree references in nodes
  for (auto& It : Nodes) {
    It.second->Tree = this;
  }
}


std::unique_ptr<LoopTransformTree> LoopTransformTree::clone() const {
  auto TreeClone = std::make_unique<LoopTransformTree>();
  TreeClone->Root = TreeClone->cloneNode(this->Root);
  // The node count will be temporarily false but that doesn't matter since all names will be copied.
  TreeClone->NodeCount = TreeClone->Nodes.size();
  assert(TreeClone->NodeCount == this->NodeCount && "The node count of the cloned tree does not match the original tree. Are there abandoned nodes?");
  return TreeClone;
}

LoopNode* LoopTransformTree::cloneNode(LoopNode* Node) {
  if (!Node)
    return nullptr;
  auto* Existing = getNode(Node->LoopName);
  if (Existing)
    return Existing;
  auto* ParentClone = cloneNode(Node->Parent);
  // Can't use make_unique here because the constructor is private
  LoopNodePtr Clone = std::unique_ptr<LoopNode>(new LoopNode(this, Node->IsVirtualLoop, Node->LoopName, ParentClone));
  auto* ClonePtr = Clone.get();
  Nodes[Clone->LoopName] = std::move(Clone);

  ClonePtr->Predecessor = cloneNode(Node->Predecessor);
  ClonePtr->Attrs = Node->Attrs;

  // Fix pointers in attributes
  for (auto& Attr : ClonePtr->Attrs.FollowupAttrs) {
    Attr.Val = cloneNode(Attr.Val);
  }

  for (auto* SubLoop : Node->SubLoops) {
    ClonePtr->SubLoops.push_back(cloneNode(SubLoop));
  }

  ClonePtr->Successor = cloneNode(Node->Successor);


  return ClonePtr;
}


LoopNode *LoopTransformTree::makeVirtualNode() {
  auto Name = std::to_string(NodeCount++);
  auto Node = new LoopNode(this, true, Name);
  Node->addStringAttribute(MDTags::LOOP_ID_TAG, Name);
  Nodes[Name].reset(Node);
  return Node;
}

LoopNode *LoopTransformTree::makeNode(std::string Name) {
  if (Name.empty()) {
    // TODO: make sure the name is unique
    Name = std::to_string(NodeCount);
  }
  NodeCount++;
  Nodes[Name] = std::unique_ptr<LoopNode>(new LoopNode(this, false, Name));
  return Nodes[Name].get();
}

void LoopTransformTree::setRoot(LoopNode *Node) {
  assert(getNode(Node->getLoopName()) == Node && "Root node is not part of the tree?");
  this->Root = Node;
}

void findValidTransformations(LoopNode* RootNode) {

}

void applyInterchange(LoopNode* RootNode) {
  if (!RootNode->isTightlyNested()) {
    for (auto& SubLoop : RootNode->subLoops()) {
      applyInterchange(&*SubLoop);
    }
    return; // TODO: Result of subloops
  }

  unsigned Depth = RootNode->getRelativeMaxDepth();

  RootNode->addBoolAttribute(MDTags::INTERCHANGE_ENABLE_TAG, true);
  RootNode->addIntAttribute(MDTags::INTERCHANGE_DEPTH_TAG, Depth);
  RootNode->addStringAttribute(MDTags::INTERCHANGE_PERMUTATION_TAG, "");
}

void applyTiling(LoopNode *RootNode) {
  if (!RootNode->isTightlyNested()) {
    for (auto& SubLoop : RootNode->subLoops()) {
      applyTiling(&*SubLoop);
    }
    return; // TODO: Result of subloops
  }

  unsigned Depth = RootNode->getRelativeMaxDepth();

  RootNode->addBoolAttribute(MDTags::TILE_ENABLE_TAG, true);
  RootNode->addIntAttribute(MDTags::TILE_DEPTH_TAG, Depth);
  RootNode->addStringAttribute(MDTags::TILE_PEEL_TAG, "rectangular");

  auto& Tree = *RootNode->getTransformTree();

  // Add metadata and create virtual successor loops
  LoopNode *Node = RootNode;
  SmallVector<LoopNode*, 3> TileLoops;
  LoopNode* LastFloor = nullptr;
  while (true) {
    Node->addIntAttribute(MDTags::TILE_SIZE_TAG, rand() % 128); // TODO: Setup loop knob here

    auto *FloorNode = Tree.makeVirtualNode();
    if (LastFloor)
      LastFloor->addSubLoop(FloorNode); // TODO: Should they even be added here as subloops? Relationship is apparent from original structure
    Node->addSuccesor(FloorNode);
    Node->addRedirectAttribute(MDTags::TILE_FOLLOWUP_FLOOR_TAG, FloorNode);
    LastFloor = FloorNode;

    auto *TileNode = Tree.makeVirtualNode();
    TileLoops.push_back(TileNode);
    Node->addRedirectAttribute(MDTags::TILE_FOLLOWUP_TILE_TAG, TileNode);

    auto *Child = Node->getFirstSubLoop(); // Perfect nesting
    if (!Child)
      break;
    Node = Child;
  }

  // Place tile loops inside the innermost floor loop.
  // NOTE: We do full tiling, i.e. the innermost tile loop may not have any subloops inherited by the original loop.
  auto* Parent = LastFloor;
  for (auto* TileNode : TileLoops) {
    Parent->addSubLoop(TileNode);
    Parent = TileNode;
  }

}

}
}
