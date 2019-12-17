//
// Created by sebastian on 16.12.19.
//

#include "LoopTransformTree.h"



namespace clang {
namespace jit {

LoopNode *LoopTransformTree::makeVirtualNode() {
  auto Name = std::to_string(NodeCount++);
  auto Node = std::make_unique<LoopNode>(this, true, Name);
  Node->addStringAttribute(MDTags::LOOP_ID_TAG, Name);
  Nodes[Name] = std::move(Node);
  return Nodes[Name].get();
}

LoopNode *LoopTransformTree::makeNode(std::string Name) {
  if (Name.empty()) {
    // TODO: make sure the name is unique
    Name = std::to_string(NodeCount);
  }
  NodeCount++;
  Nodes[Name] = std::make_unique<LoopNode>(this, false, Name);
  return Nodes[Name].get();
}



void applyTiling(LoopNode *RootNode) {
  if (!RootNode->isPerfectlyNested()) {
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
    Node->addIntAttribute(MDTags::TILE_SIZE_TAG, 128); // TODO: Setup loop knob here

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
