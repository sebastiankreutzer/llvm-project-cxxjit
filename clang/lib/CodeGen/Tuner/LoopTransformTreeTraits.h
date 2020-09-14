//
// Created by sebastian on 18.12.19.
//

#ifndef LLVM_LOOPTRANSFORMTREETRAITS_H
#define LLVM_LOOPTRANSFORMTREETRAITS_H

#include "llvm/ADT/GraphTraits.h"
#include "llvm/Support/DOTGraphTraits.h"
#include <sstream>

#include "LoopTransformTree.h"

namespace llvm {

using namespace clang;
using namespace clang::jit;

template<>
struct GraphTraits<LoopTransformTree*> {
  // Elements to provide:

  // typedef NodeRef           - Type of Node token in the graph, which should
  //                             be cheap to copy.
  using NodeRef = LoopNode*;

  // typedef ChildIteratorType - Type used to iterate over children in graph,
  //                             dereference to a NodeRef.
  using ChildIteratorType = LoopNode::LoopList::iterator;

  // static NodeRef getEntryNode(const GraphType &)
  //    Return the entry node of the graph
  static NodeRef getEntryNode(const LoopTransformTree& Tree) {
    return Tree.getRoot();
  }

  // static ChildIteratorType child_begin(NodeRef)
  // static ChildIteratorType child_end  (NodeRef)
  //    Return iterators that point to the beginning and ending of the child
  //    node list for the specified node.

  static ChildIteratorType child_begin(NodeRef Node) {
    return Node->subLoops().begin();
  }
  static ChildIteratorType child_end(NodeRef Node) {
    return Node->subLoops().end();
  }

  // typedef  ...iterator nodes_iterator; - dereference to a NodeRef
  // static nodes_iterator nodes_begin(GraphType *G)
  // static nodes_iterator nodes_end  (GraphType *G)
  //    nodes_iterator/begin/end - Allow iteration over all nodes in the graph

  using nodes_iterator = util::DereferenceIterator<LoopTransformTree::node_iterator>;
  static nodes_iterator nodes_begin(LoopTransformTree* Tree) {
    return dereference_iterator(Tree->nodes_begin());
  }
  static nodes_iterator nodes_end(LoopTransformTree* Tree) {
    return dereference_iterator(Tree->nodes_end());
  }

  // typedef EdgeRef           - Type of Edge token in the graph, which should
  //                             be cheap to copy.
  // typedef ChildEdgeIteratorType - Type used to iterate over children edges in
  //                             graph, dereference to a EdgeRef.

  using EdgeRef = NodeRef;
  using ChildEdgeIteratorType = ChildIteratorType;

  // static ChildEdgeIteratorType child_edge_begin(NodeRef)
  // static ChildEdgeIteratorType child_edge_end(NodeRef)
  //     Return iterators that point to the beginning and ending of the
  //     edge list for the given callgraph node.
  //
  // static NodeRef edge_dest(EdgeRef)
  //     Return the destination node of an edge.
  static ChildEdgeIteratorType child_edge_begin(NodeRef Node) {
    return Node->subLoops().begin();
  }
  static ChildEdgeIteratorType child_edge_end(NodeRef Node) {
    return Node->subLoops().end();
  }

  // static unsigned       size       (GraphType *G)
  //    Return total number of nodes in the graph
  static unsigned size(LoopTransformTree* Tree) {
    return Tree->size();
  }

};


template <>
struct DOTGraphTraits<LoopTransformTree*> : public DefaultDOTGraphTraits {

  DOTGraphTraits (bool isSimple=false) : DefaultDOTGraphTraits(isSimple) {}

  std::string getNodeLabel(const LoopNode * Node, const LoopTransformTree*) {
    std::stringstream ss;
    ss << Node->getLoopName().str() << "\n";
    ss << "Trip count: ";
    if (Node->getTripCountInfo().IsExact)
      ss << Node->getTripCountInfo().TripCount;
    else
      ss << "Unknown";
    auto Succ = Node->getSuccessor();
    if (Succ)
      ss << "\nSucc: " << Succ->getLoopName().str();
    return ss.str();
  }

};



}


#endif //LLVM_LOOPTRANSFORMTREETRAITS_H
