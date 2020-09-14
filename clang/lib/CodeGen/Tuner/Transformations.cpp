//
// Created by sebastian on 20.12.19.
//

#include "LoopTransformTreeTraits.h"
#include "llvm/Support/GraphWriter.h"
#include "Transformations.h"
#include "Debug.h"

namespace clang {
namespace jit {

void viewTree(LoopTransformTree& Tree) {
  ViewGraph(&Tree, "LoopTree");
}

void findUnrollTransformations(LoopNode* Root, SmallVectorImpl<LoopTransformation>& Transformations) {

  // TODO: Right now, each loop unroll is treated as a single transformation.
  //  It might be better to tune them together, will have to try that out.

  // TODO: Unrolling only on last-level loop sensible?

  LoopNode* Node = Root->getLastSuccessor();
  // Avoid unrolling floor loops from preceding tiling transformation, as well as previously unrolled loops
  if (!Node->hasSubLoop() && !(Node->isSetInPredecessors(LoopNode::TILED_FLOOR) || Node->isSetInPredecessors(LoopNode::UNROLLED))) {
    LoopTransformation Trans;
    Trans.Root = Node->getLoopName();
    Trans.Kind = LoopTransformation::UNROLL;
    auto Name = ("Loop " + Node->getLoopName() + " - Unroll factor").str();
    auto& TTI = Node->getTripCountInfo();
    auto Min = transform_defaults::UNROLL_MIN;
    // Avoid extremely high unroll counts
    auto Max = TTI.hasInfo() ? std::min(TTI.TripCount, transform_defaults::UNROLL_MAX) : transform_defaults::UNROLL_MAX;
    auto Dflt = std::max(Min, Max / 4);

    // TODO: For small trip counts, only try full unrolling


    if (Max == transform_defaults::UNROLL_MIN) {
      assert(Min == Max);
      // Do nothing
    } else {
      Trans.addSearchDim(SearchDim(Min, Max, Dflt, Name));
      Transformations.push_back(Trans);
    }
  }
  for (auto& SL : Node->subLoops()) {
    findUnrollTransformations(SL, Transformations);
  }
}


void findUnrollAndJamTransformations(LoopNode* Root, SmallVectorImpl<LoopTransformation>& Transformations) {
  Root = Root->getLastSuccessor();

  if (!Root->hasSubLoop())
    return;

  if (!Root->isTightlyNested()) {
    for (auto& SubLoop : Root->subLoops()) {
      findUnrollAndJamTransformations(&*SubLoop, Transformations);
    }
    return;
  }

  LoopNode* Node = Root;
  // Avoid unrolling floor loops from preceding tiling transformation, as well as previously unrolled loops
  while(Node->isSetInPredecessors(LoopNode::TILED_FLOOR) || Node->isSetInPredecessors(LoopNode::UNROLLED)) {
    Node = Node->getFirstSubLoop();
    if (!Node)
      return;
    Node = Node->getLastSuccessor();
  }

  if (!Node->hasSubLoop())
    return;

  unsigned Depth = Node->getRelativeMaxDepth();
  LoopTransformation Trans;
  Trans.Root = Node->getLoopName();
  Trans.Kind = LoopTransformation::UNROLL_AND_JAM;

  int BaseMax = 32 / Depth;

  // Last-level loop is the one that is jammed.
  for (unsigned i = 0; i < Depth-1; i++) {
    assert(Node && "Node is null");
    Node = Node->getLastSuccessor();
    // For simplicity, none of the inner loops may be unrolled before
    if (Node->isSetInPredecessors(LoopNode::UNROLLED))
      return;
    auto Name = ("Loop " + Node->getLoopName() + " - Unroll(-and-jam) factor").str();
    // TODO: Should loops with disabled unrolling be split in separate transformations?
    //  They don't modify the structure of the loop but given uniform sampling, it is very likely that all loops will be unrolled in most configurations.
    auto& TTI = Node->getTripCountInfo();
    auto Min = transform_defaults::UNROLL_AND_JAM_MIN;


    int Level = Depth - 1 - i;
    auto LevelMax = transform_defaults::UNROLL_AND_JAM_MAX;
    if (Level == 2)
      LevelMax = 8;
    else if (Level >= 3) {
      LevelMax = 2;
    }
    //outs() << "Level max is " << LevelMax << "\n";
    auto Max = TTI.hasInfo() ? std::min(TTI.TripCount, LevelMax) : LevelMax;

    auto Dflt = std::max(Min, Max / 4);

    if (Max == transform_defaults::UNROLL_AND_JAM_MIN) {
      assert(Min == Max);
      Trans.IntParams.push_back(i);
      Trans.FixedParams.push_back(Min);
    } else {
      Trans.addSearchDim(SearchDim(Min, Max, Dflt, Name));
    }

    Node = Node->getFirstSubLoop();
  }
  if (!Trans.Space.empty())
    Transformations.push_back(Trans);
}

static SmallVector<SmallVector<int, 4>, 4> listPermutations(ArrayRef<int> Vals) {
  assert(!Vals.empty());
  if (Vals.size() == 1) {
    return {{Vals.front()}};
  }
  SmallVector<SmallVector<int, 4>, 4> Perms;
  for (int i = 0; i < Vals.size(); i++) {
    auto Val = Vals[i];
    SmallVector<int, 4> Remaining;
    for (int j = 0; j < Vals.size(); j++) {
      if (i != j)
        Remaining.push_back(Vals[j]);
    }
    for (auto& Tail : listPermutations(Remaining)) {
      SmallVector<int, 4> Perm;
      Perm.push_back(Val);
      Perm.insert(Perm.end(), Tail.begin(), Tail.end());
      Perms.push_back(Perm);
    }
  }
  return Perms;
}

static int factorial(int n) {
  return n <= 1 ? 1 : n * factorial(n-1);
}

void findInterchangeTransformationsSeparate(LoopNode* Root, SmallVectorImpl<LoopTransformation> & Transformations) {
  Root = Root->getLastSuccessor();

  if (!Root->hasSubLoop())
    return;

  if (!Root->isTightlyNested()) {
    for (auto& SubLoop : Root->subLoops()) {
      findInterchangeTransformationsSeparate(&*SubLoop, Transformations);
    }
    return;
  }



  unsigned Depth = Root->getRelativeMaxDepth();

  SmallVector<LoopNode*, 4> Nodes;

  LoopNode* Node = Root;
  SmallVector<int,4> Indices;
  int Barrier = -1;
  for (unsigned i = 0; i < Depth; i++) {
    assert(Node && "Node is null");
    Node = Node->getLastSuccessor();
    if (Node->isSetInPredecessors(LoopNode::INTERCHANGED))
      return;
    Indices.push_back(i);
    if (Node->hasInterchangeBarrier())
      Barrier = i;
    Nodes.push_back(Node);
    Node = Node->getFirstSubLoop();
  }

  auto violatesConstraints = [&](ArrayRef<int> Perm) -> bool {
    for (int I = 0; I < Barrier; I++) {
      if (Perm[I] >= Barrier)
        return true;
    }
    return false;
  };

  auto Perms = listPermutations(Indices);
  assert(Perms.size() == factorial(Depth) && "Wrong number of permutations");
  auto NumValid = 0;
  for (auto& Perm : Perms) {
    if (violatesConstraints(Perm)) {

      continue;
    }
//    outs() << "Found permutation: " << "\n";
//    for (auto Val : Perm) {
//      outs() << Val << ",";
//    }
//    outs() << "\n";
    LoopTransformation Trans;
    Trans.Root = Root->getLoopName();
    Trans.Kind = LoopTransformation::INTERCHANGE;
    Trans.IntParams = Perm;
    Transformations.push_back(std::move(Trans));
    NumValid++;
  }
  //outs() << "Found " << NumValid << " valid permutations\n";
}

//void findInterchangeTransformations(LoopNode* Root, SmallVectorImpl<LoopTransformation> & Transformations) {
//  Root = Root->getLastSuccessor();
//
//  if (!Root->hasSubLoop())
//    return;
//
//  if (!Root->isTightlyNested()) {
//    for (auto& SubLoop : Root->subLoops()) {
//      findInterchangeTransformations(&*SubLoop, Transformations);
//    }
//    return;
//  }
//
//  unsigned Depth = Root->getRelativeMaxDepth();
//  LoopTransformation Trans;
//  Trans.Root = Root->getLoopName();
//  Trans.Kind = LoopTransformation::INTERCHANGE;
//  LoopNode* Node = Root;
//  for (unsigned i = 1; i <= Depth; i++) {
//    assert(Node && "Node is null");
//    Node = Node->getLastSuccessor();
//    if (Node->isSetInPredecessors(LoopNode::INTERCHANGED))
//      return;
//    auto Name = ("Loop " + Node->getLoopName() + " - Interchange priority").str();
//    // TODO: Interchange permutation is a structural transformation, should not be controlled with knobs!
//    //  Instead enumerate all legal permutations, respecting dependencies induced by previous transformations
//    IntKnob* InterchangePriority = new IntKnob(1, 256, 1, std::move(Name)); // TODO: Heuristic for max value (also memory leak)
//    Trans.addKnob(InterchangePriority, "interchange");
//    Node = Node->getFirstSubLoop();
//  }
//  Transformations.push_back(std::move(Trans));
//}


void findTilingTransformations(LoopNode *Root, SmallVectorImpl<LoopTransformation> &Transformations) {
  Root = Root->getLastSuccessor();
  if (!Root->isTightlyNested()) {
    for (auto& SubLoop : Root->subLoops()) {
      findTilingTransformations(&*SubLoop, Transformations);
    }
    return;
  }
  unsigned Depth = Root->getRelativeMaxDepth();
  LoopTransformation Trans;
  Trans.Root = Root->getLoopName();
  Trans.Kind = LoopTransformation::TILE;
  LoopNode* Node = Root;
  for (unsigned i = 1; i <= Depth; i++) {
    assert(Node && "Node is null");
    Node = Node->getLastSuccessor();
    if (Node->isSetInPredecessors(LoopNode::UNROLLED) || Node->isSetInPredecessors(LoopNode::TILED_TILE) || Node->isSetInPredecessors(LoopNode::TILED_FLOOR))
      return;
    auto Name = ("Loop " + Node->getLoopName() + " - Tile Size").str();
    auto& TTI = Node->getTripCountInfo();

    // A tile size of one disables tiling for the corresponding loop.

    unsigned Min = transform_defaults::TILE_MIN;
    unsigned Max = Min;
    unsigned Dflt = Min;

    if (TTI.hasInfo()) {
      if (TTI.TripCount >= transform_defaults::MIN_TILING_TRIP_COUNT) {
        Max = TTI.TripCount / 2;
        Dflt = std::max(Min, Max / 4);
      }
    } else {
      // Exact trip count is unknown.
      Max = transform_defaults::TILE_MAX;
      Dflt = transform_defaults::TILE_DFLT;
    }

    if (Max == transform_defaults::TILE_MIN) {
      assert(Min == Max);
      // Save depth and fixed parameter value.
      Trans.IntParams.push_back(i-1);
      Trans.FixedParams.push_back(ParamVal(Min));
    } else {
      Trans.addSearchDim(SearchDim(Min, Max, Dflt, Name));
    }
    Node = Node->getFirstSubLoop();
  }
  // Only register the transformation if at least one loop can be tiled.
  if (!Trans.Space.empty())
    Transformations.push_back(std::move(Trans));
}

void findTransformations(LoopNode* Root, SmallVectorImpl<LoopTransformation>& Transformations) {
  findTilingTransformations(Root, Transformations);
  findInterchangeTransformationsSeparate(Root, Transformations);
  findUnrollAndJamTransformations(Root, Transformations);
  //findUnrollTransformations(Root, Transformations);
}

SmallVector<LoopTransformation, 4> findTransformations(LoopTransformTree *Tree) {
  assert(Tree && "Tree is null");
  SmallVector<LoopTransformation, 4> Transformations;
  auto Root = Tree->getRoot();
  if (Root) {
    findTransformations(Root, Transformations);
  }
  return Transformations;
}


void apply(LoopTransformation& Transformation, LoopTransformTree& Tree, ParamConfig& Cfg) {

  auto FetchFixedAndTunableIntParams = [&Transformation, &Cfg](SmallVectorImpl<unsigned>& Params) {
    unsigned Depth = Cfg.size() + Transformation.FixedParams.size();
    assert(Transformation.IntParams.size() == Transformation.FixedParams.size() && "IntParams must store loop indices");
    auto FixedIt = Transformation.IntParams.begin();
    auto FixedValIt = Transformation.FixedParams.begin();
    for (unsigned I = 0; I < Depth; I++) {
      if (FixedIt != Transformation.IntParams.end() && *FixedIt == I) {
        Params.push_back(cantFail((FixedValIt++)->getIntVal()));
        FixedIt++;
      } else {
        Params.push_back(cantFail(Cfg[I].getIntVal()));
      }
    }
  };

  if (Transformation.Kind == LoopTransformation::NONE) {
    return;
  }
  auto Root = Tree.getNode(Transformation.Root);
  assert(Root && "Root node of transformation does not exist in tree");
  switch(Transformation.Kind) {
    case LoopTransformation::TILE: {

      // Collect tile sizes from tunable and fixed parameters.
      SmallVector<unsigned, 4> Sizes;
      FetchFixedAndTunableIntParams(Sizes);
      auto Depth = Sizes.size();

      // FIXME: Polly triggers an assertion when the trip count is a multiple of the tile size.
      //  This is a workaround that should be removed ASAP.
//      auto* Node = Root;
//      bool Change = true;
//      for (auto i = 0; i < Sizes.size(); i++) {
//        Node = Node->getLastSuccessor();
//        if (!Node->getOriginalLoop()->getTripCountInfo().hasInfo() || Node->getOriginalLoop()->getTripCountInfo().TripCount % Sizes[i] != 0) {
//          //errs() << "No change: " << Node->getTripCountInfo().TripCount << "\n";
//          Change = false;
//          break;
//        }
//        if (Node->getFirstSubLoop())
//          Node = Node->getFirstSubLoop();
//      }
//      if (Change) {
//        auto& Size = Sizes.back();
//        while (Node->getOriginalLoop()->getTripCountInfo().TripCount % Size == 0) {
//          JIT_INFO(errs() << "Warning: Changing tile size of loop from " << Size << " to " << Size + 1
//                 << " to work around polly bug\n");
//          Size++;
//        }
//      }

      assert(Depth > 0);
      applyTiling(Root, Depth, Sizes);
      break;
    }
    case LoopTransformation::INTERCHANGE: {

      auto Permutation = Transformation.IntParams;

      unsigned Depth = Permutation.size();
      assert(Depth > 0);
      applyInterchange(Root, Depth, Permutation);
      break;
    }
    case LoopTransformation::UNROLL_AND_JAM: {
      SmallVector<unsigned, 4> Counts;
      // Collect unroll counts from tunable and fixed parameters.
      FetchFixedAndTunableIntParams(Counts);

      applyUnrollAndJam(Root, Counts);
      break;
    }
    case LoopTransformation::UNROLL: {
//      ArrayRef<KnobID> CountKnobs = Transformation.getKnobs("unroll");
      SmallVector<unsigned, 4> Counts;
      // Collect unroll counts from tunable and fixed parameters
      // (should only be tunable in this case, since each loop is unrolled separately).
      FetchFixedAndTunableIntParams(Counts);

      applyUnroll(Root, Counts);
      break;
    }
    default:
      llvm_unreachable("Invalid transformation kind.");
  }
}

void applyUnroll(LoopNode* Root, ArrayRef<unsigned> Counts) {
  assert(Root && "Root is null");
  assert(Counts.size() == 1 && "Unroll transformations only implemented for single loops");

  auto& Tree = *Root->getTransformTree();

  auto* Node = Root->getLastSuccessor();
  unsigned Count = Counts.front();
  bool DoUnroll = Count > 1;
  if (DoUnroll) {
    bool Full = Node->getTripCountInfo().IsExact && Node->getTripCountInfo().TripCount == Count;
    Node->addBoolAttribute(MDTags::UNROLL_ENABLE_TAG, true);
    if (Full) {
      Node->addBoolAttribute(MDTags::UNROLL_FULL_TAG, true);
    } else {
      Node->addIntAttribute(MDTags::UNROLL_COUNT_TAG, Count);
    }
    auto Unrolled = Tree.makeVirtualNode();
    Unrolled->setFlag(LoopNode::UNROLLED);
    Unrolled->getTripCountInfo() = Node->getTripCountInfo();
    Unrolled->getTripCountInfo().TripCount /= Count;
    Node->addSuccesor(Unrolled);
    // TODO: For full unroll, modify graph correctly
    // TODO: Followup for simple unroll not implemented yet
  } else {
    // Loops are marked as unrolled, even if no unrolling actually occurs.
    Node->setFlag(LoopNode::UNROLLED);
  }

}

void applyUnrollAndJam(LoopNode* Root, ArrayRef<unsigned> Counts) {
  assert(Root && "Root is null");
  assert(Root->isTightlyNested() && "Root not tightly nested"); // TODO: Actually needs perfect nesting, check for that

  auto& Tree = *Root->getTransformTree();

  auto Node = Root;
  for (auto Count : Counts) {
    assert(Node && "Unroll-and-jam not compatible with loop structure");
    Node = Node->getLastSuccessor();
    bool DoUnroll = Count > 1;
    if (DoUnroll) {
      Node->addBoolAttribute(MDTags::UNROLL_AND_JAM_ENABLE_TAG, true);
      Node->addIntAttribute(MDTags::UNROLL_AND_JAM_COUNT_TAG, Count);
      // NOTE: We ignore remainder loops, as they are usually unrolled fully.
      auto Unrolled = Tree.makeVirtualNode();
      Unrolled->setFlag(LoopNode::UNROLLED);
      Unrolled->getTripCountInfo() = Node->getTripCountInfo();
      Unrolled->getTripCountInfo().TripCount /= Count;
      Node->addSuccesor(Unrolled);
      Node->addRedirectAttribute(MDTags::UNROLL_AND_JAME_FOLLOWUP_UNROLLED_TAG, Unrolled);
    } else {
      // Loops are marked as unrolled, even if no unrolling actually occurs.
      Node->setFlag(LoopNode::UNROLLED);
    }
    Node = Node->getFirstSubLoop();

  }
}

void applyInterchange(LoopNode *Root, unsigned Depth, ArrayRef<int> Permutation) {
  assert(Root && "Root is null");
  assert(Root->isTightlyNested() && "Root not tightly nested"); // TODO: Actually needs perfect nesting, check for that
  assert(Depth == Permutation.size());

  Root->addBoolAttribute(MDTags::INTERCHANGE_ENABLE_TAG, true);
  Root->addIntAttribute(MDTags::INTERCHANGE_DEPTH_TAG, Depth);
  Root->addIntListAttribute(MDTags::INTERCHANGE_PERMUTATION_TAG, Permutation);

  auto& Tree = *Root->getTransformTree();

  if (Depth > 2) {
//    ViewGraph(&Tree, "Interchange graph");
//    outs() << "Interchange with depth > 2\n";
  }

  SmallVector<SmallVector<int, 2>, 4> Constraints;

  SmallVector<LoopNode*, 4> InterchangedLoops(Depth, nullptr);

  // Create virtual interchanged successor loops
  LoopNode *Node = Root;
  for (unsigned I = 0; I < Depth; I++) {
    assert(Node && "Interchange not compatible with loop structure");
    Node = Node->getLastSuccessor();
    // Save constraints
//    auto IC = Node->getInterchangeConstraints();
//    Constraints.push_back(IC);

    int LoopIdx = Permutation[I];
    auto Interchanged = Tree.makeVirtualNode();
    Interchanged->setFlag(LoopNode::INTERCHANGED);
    Interchanged->getTripCountInfo() = Node->getTripCountInfo();
    Node->addSuccesor(Interchanged);
    Node->addRedirectAttribute(MDTags::INTERCHANGE_FOLLOWUP_TAG, Interchanged);
    InterchangedLoops[LoopIdx] = Interchanged;
    Node = Node->getFirstSubLoop();
  }

  // Fix loop nesting
  auto* LastNode = Root->getParent();
  for (int i = 0; i < Depth; i++) {
    auto Node = InterchangedLoops[i];
//    Node->setInterchangeConstraints(Constraints)
    if (LastNode) {
      LastNode->addSubLoop(Node);
    }
    LastNode = Node;
  }
}

void applyTiling(LoopNode *Root, unsigned Depth, ArrayRef<unsigned> Sizes, StringRef PeelType) {
  assert(Root && "Root is null");
  assert(Root->isTightlyNested() && "Root not tightly nested"); // TODO: Actually needs perfect nesting, check for that
  assert(Depth == Sizes.size());

  Root->addBoolAttribute(MDTags::TILE_ENABLE_TAG, true);
  Root->addIntAttribute(MDTags::TILE_DEPTH_TAG, Depth);
  // Root->addStringAttribute(MDTags::TILE_PEEL_TAG, PeelType); // FIXME: Currently leads to a bug in polly

  auto& Tree = *Root->getTransformTree();

  // Add metadata and create virtual successor loops
  LoopNode *Node = Root;
  SmallVector<LoopNode*, 3> TileLoops;
  LoopNode* LastFloor = nullptr;

  for (unsigned I = 0; I < Depth; I++) {
    assert(Node && "Given tiling not compatible with loop structure");
    Node = Node->getLastSuccessor();

    unsigned TileSize = Sizes[I];
    Node->addIntAttribute(MDTags::TILE_SIZE_TAG, TileSize);

    auto& OriginalTripCount = Node->getTripCountInfo();

    auto *FloorNode = Tree.makeVirtualNode();
    if (LastFloor)
      LastFloor->addSubLoop(FloorNode); // TODO: Should they even be added here as subloops? Relationship is apparent from original structure
    Node->addSuccesor(FloorNode);
    Node->addRedirectAttribute(MDTags::TILE_FOLLOWUP_FLOOR_TAG, FloorNode);
    FloorNode->getTripCountInfo() = OriginalTripCount;
    FloorNode->getTripCountInfo().TripCount /= TileSize;
    LastFloor = FloorNode;

    auto *TileNode = Tree.makeVirtualNode();
    TileNode->getTripCountInfo() = LoopNode::TripCountInfo(TileSize, true);
    TileLoops.push_back(TileNode);
    Node->addRedirectAttribute(MDTags::TILE_FOLLOWUP_TILE_TAG, TileNode);

    TileNode->setFlag(LoopNode::TILED_TILE);
    FloorNode->setFlag(LoopNode::TILED_FLOOR);

    Node = Node->getFirstSubLoop(); // Perfect nesting

  }

  TileLoops.front()->setInterchangeBarrier();

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