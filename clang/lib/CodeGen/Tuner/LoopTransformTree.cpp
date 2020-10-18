//
// Created by sebastian on 16.12.19.
//

#include "LoopTransformTree.h"



namespace clang {
namespace jit {

namespace MDTags {
//const char *LOOP_ID_TAG = "llvm.loop.id";
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
const char *INTERCHANGE_FOLLOWUP_TAG = "llvm.loop.interchange.followup_interchanged";
const char *UNROLL_AND_JAM_ENABLE_TAG = "llvm.loop.unroll_and_jam.enable";
const char *UNROLL_AND_JAM_COUNT_TAG = "llvm.loop.unroll_and_jam.count";
const char *UNROLL_AND_JAME_FOLLOWUP_UNROLLED_TAG = "llvm.loop.unroll_and_jam.followup_outer_unrolled";
const char *UNROLL_ENABLE_TAG = "llvm.loop.unroll.enable";
const char *UNROLL_COUNT_TAG = "llvm.loop.unroll.count";
const char *UNROLL_FULL_TAG = "llvm.loop.unroll.full";

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

  // First create a copy of all nodes
  for (auto& It : Nodes) {
    auto& Node = It.second;
    LoopNodePtr Clone = std::unique_ptr<LoopNode>(new LoopNode(TreeClone.get(), Node->IsVirtualLoop, Node->LoopName, nullptr));
    Clone->Flags = Node->Flags;
    Clone->TCI = Node->TCI;
    Clone->InterchangeBarrier = Node->InterchangeBarrier;
    TreeClone->Nodes[Clone->LoopName] = std::move(Clone);
  }

  auto getClonedNode = [&TreeClone](LoopNode* Original) -> LoopNode* {
    if (Original)
      return TreeClone->getNode(Original->LoopName);
    return nullptr;
  };

  // Then fix edges, cross-references etc.
  for (auto& It : TreeClone->Nodes) {
    auto& Node = It.second;
    auto* OrigNode = getNode(It.first());
    Node->Parent = getClonedNode(OrigNode->Parent);

    Node->Predecessor = getClonedNode(OrigNode->Predecessor);
    for (auto *SubLoop : OrigNode->SubLoops) {
      Node->SubLoops.push_back(getClonedNode(SubLoop));
    }

    Node->Successor = getClonedNode(OrigNode->Successor);

    Node->Attrs = OrigNode->Attrs;

    // Fix pointers in attributes
    for (auto& Attr : Node->Attrs.FollowupAttrs) {
      Attr.Val = getClonedNode(Attr.Val);
    }

  }

  TreeClone->Root = getClonedNode(getRoot());
  // The node count will be temporarily false but that doesn't matter since all names will be copied.
  TreeClone->NodeCount = TreeClone->Nodes.size();
  assert(TreeClone->NodeCount == this->NodeCount && "The node count of the cloned tree does not match the original tree. Are there abandoned nodes?");
  return TreeClone;
}


LoopNode *LoopTransformTree::makeVirtualNode() {
  auto Name = std::to_string(NodeCount++);
  auto Node = new LoopNode(this, true, Name);
  Node->addStringAttribute(LOOP_NAME_TAG, Name);
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


}
}
