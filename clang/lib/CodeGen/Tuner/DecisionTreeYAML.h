//
// Created by sebastian on 01.10.20.
//

#ifndef CLANG_DECISIONTREEYAML_H
#define CLANG_DECISIONTREEYAML_H

#include "llvm/Support/YAMLTraits.h"

#include "TransformTreeOptimizer.h"

using llvm::yaml::MappingTraits;
using llvm::yaml::SequenceTraits;
using llvm::yaml::ScalarTraits;
using llvm::yaml::ScalarEnumerationTraits;
using llvm::yaml::IO;

using namespace clang::jit;

template <unsigned U> struct ScalarTraits<SmallString<U>> {

  static void output(const SmallString<U> &S, void *, llvm::raw_ostream &OS) {
    for (const auto &C : S)
      OS << C;
  }

  static StringRef input(StringRef Scalar, void *, SmallString<U> &Value) {
    Value.assign(Scalar.begin(), Scalar.end());
    return StringRef();
  }

  static QuotingType mustQuote(StringRef) { return QuotingType::Single; }
};


LLVM_YAML_IS_SEQUENCE_VECTOR(ParamVal);

template<>
struct ScalarEnumerationTraits<LoopTransformation::TransformKind> {
  static void enumeration(IO& io, LoopTransformation::TransformKind& Kind) {
    io.enumCase(Kind, "None", LoopTransformation::NONE);
    io.enumCase(Kind, "Tile", LoopTransformation::TILE);
    io.enumCase(Kind, "Interchange", LoopTransformation::INTERCHANGE);
    io.enumCase(Kind, "Unroll", LoopTransformation::UNROLL);
    io.enumCase(Kind, "Unroll-and-jam", LoopTransformation::UNROLL_AND_JAM);
    io.enumCase(Kind, "Array-pack", LoopTransformation::ARRAY_PACK);
    io.enumCase(Kind, "Vectorize", LoopTransformation::VECTORIZE);
  }
};

template<>
struct MappingTraits<LoopTransformation> {
  static void mapping(IO& io, LoopTransformation& Trans) {
    io.mapRequired("kind", Trans.Kind);
    io.mapRequired("fixed-params", Trans.FixedParams);
    io.mapRequired("int-params", Trans.IntParams);
    io.mapRequired("search-space", Trans.Space);
    io.mapRequired("root", Trans.Root);
  }
};

template<>
struct SequenceTraits<SearchSpace> {
  static size_t size(IO& io, SearchSpace& Space) {
    return Space.getNumDimensions();
  }
  static SearchDim& element(IO& io, SearchSpace& Space, size_t Index) {
    return Space[Index];
  }
};

template<>
struct ScalarEnumerationTraits<ParamType> {
  static void enumeration(IO& io, ParamType& Type) {
    io.enumCase(Type, "FP", ParamType::FP);
    io.enumCase(Type, "INT", ParamType::INT);
  }
};

template<>
struct ScalarTraits<ParamVal> {
  static void output(const ParamVal& Val, void*, llvm::raw_ostream& OS) {
    if (Val.Type==ParamType::FP) {
      auto FPVal = Val.getFPVal();
      if (FPVal)
        OS << FPVal.get();
      else
        OS << "ERR";
    } else {
      auto IntVal = Val.getIntVal();
      if (IntVal)
        OS << IntVal.get();
      else
        OS << "ERR";
    }
  }

  static StringRef input(StringRef Scalar, void*, ParamVal& Val) {
    // TODO: Implement ParamVal read
    return StringRef();
  }

  static QuotingType mustQuote(StringRef) {
    return QuotingType::None;
  }
};

template<>
struct MappingTraits<SearchDim> {
  static void mapping(IO& io, SearchDim& Dim) {
    io.mapRequired("name", Dim.Name);
    io.mapRequired("type", Dim.Type);
    io.mapRequired("min", Dim.Min);
    io.mapRequired("max", Dim.Max);
    io.mapRequired("dflt", Dim.Default);
  }
};

template<>
struct MappingTraits<DecisionNode> {
  static void mapping(IO& io, DecisionNode& Node) {
    io.mapRequired("transformation",  Node.Transformation);
    io.mapRequired("children", Node.Children);
  }
};

template<>
struct MappingTraits<std::unique_ptr<DecisionNode>> {
  static void mapping(IO& io, std::unique_ptr<DecisionNode>& Ptr) {
    if (Ptr)
      MappingTraits<DecisionNode>::mapping(io, *Ptr);
  }
};

LLVM_YAML_IS_SEQUENCE_VECTOR(std::unique_ptr<DecisionNode>);

//
//template<unsigned N>
//struct SequenceTraits<SmallVector<std::unique_ptr<DecisionNode>, N>> {
//  static size_t size(IO& io, SmallVector<std::unique_ptr<DecisionNode>, N>& Nodes) {
//    return Nodes.size();
//  }
//  static std::unique_ptr<DecisionNode>& element(IO& io, SmallVector<std::unique_ptr<DecisionNode>, N>& Nodes, size_t Index) {
//    return Nodes[Index];
//  }
//};

inline void writeTree(TransformDecisionTree& Tree, raw_ostream& OS=llvm::outs()) {
  llvm::yaml::Output Out(OS);
  Out << Tree.getRoot();
}

inline void writeTree(TransformDecisionTree& Tree, StringRef File) {
  std::error_code Err;
  raw_fd_ostream OS(File, Err);
  writeTree(Tree, OS);
}


#endif // CLANG_DECISIONTREEYAML_H
