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

//namespace llvm {
//namespace yaml {

template<unsigned U>
struct ScalarTraits<SmallString<U>>
{

  static void output(const SmallString<U> &S, void *, llvm::raw_ostream &OS)
  {
    for (const auto &C : S)
      OS << C;
  }

  static StringRef input(StringRef Scalar, void *, SmallString<U> &Value)
  {
    Value.assign(Scalar.begin(), Scalar.end());
    return StringRef();
  }

  static QuotingType mustQuote(StringRef)
  { return QuotingType::Single; }
};


LLVM_YAML_IS_SEQUENCE_VECTOR(ParamVal);

template<>
struct llvm::yaml::ScalarEnumerationTraits<LoopTransformation::TransformKind>
{
  static void enumeration(IO &io, LoopTransformation::TransformKind &Kind)
  {
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
struct llvm::yaml::MappingTraits<LoopTransformation>
{
  static void mapping(IO &io, LoopTransformation &Trans)
  {
    io.mapRequired("kind", Trans.Kind);
    io.mapRequired("fixed-params", Trans.FixedParams);
    io.mapRequired("int-params", Trans.IntParams);
    io.mapRequired("search-space", Trans.Space);
    io.mapRequired("root", Trans.Root);
  }
};

template<>
struct llvm::yaml::SequenceTraits<SearchSpace>
{
  static size_t size(IO &io, SearchSpace &Space)
  {
    return Space.getNumDimensions();
  }

  static SearchDim &element(IO &io, SearchSpace &Space, size_t Index)
  {
    return Space[Index];
  }
};

template<>
struct llvm::yaml::ScalarEnumerationTraits<ParamType>
{
  static void enumeration(IO &io, ParamType &Type)
  {
    io.enumCase(Type, "FP", ParamType::FP);
    io.enumCase(Type, "INT", ParamType::INT);
  }
};

template<>
struct llvm::yaml::ScalarTraits<ParamVal>
{
  static void output(const ParamVal &Val, void *, llvm::raw_ostream &OS)
  {
    if (Val.Type == ParamType::FP) {
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

  static StringRef input(StringRef Scalar, void *, ParamVal &Val)
  {
    // TODO: Implement ParamVal read
    return StringRef();
  }

  static QuotingType mustQuote(StringRef)
  {
    return QuotingType::None;
  }
};

template<>
struct llvm::yaml::MappingTraits<SearchDim>
{
  static void mapping(IO &io, SearchDim &Dim)
  {
    io.mapRequired("name", Dim.Name);
    io.mapRequired("type", Dim.Type);
    io.mapRequired("min", Dim.Min);
    io.mapRequired("max", Dim.Max);
    io.mapRequired("dflt", Dim.Default);
  }
};

template<>
struct llvm::yaml::MappingTraits<ConfigEval>
{
  static void mapping(IO& io, ConfigEval& Eval) {
    io.mapRequired("stats", Eval.Stats);
    io.mapRequired("config", Eval.Config.Values);
  }
};

LLVM_YAML_IS_SEQUENCE_VECTOR(ConfigEval);

template<>
struct llvm::yaml::MappingTraits<SharedEvalStats>
{
  static void mapping(IO& io, SharedEvalStats& Stats) {
    if (Stats) {
      io.mapRequired("num", Stats->N);
      io.mapRequired("mean", Stats->Mean);
      io.mapRequired("variance", Stats->Variance);
    }
  }
};

template<>
struct llvm::yaml::SequenceTraits<ParamConfig::VecT>
{
  static size_t size(IO &io, ParamConfig::VecT& V)
  {
    return V.size();
  }

  static ParamVal &element(IO &io, ParamConfig::VecT& V, size_t Index)
  {
    return V[Index];
  }
};

template<>
struct llvm::yaml::MappingTraits<DecisionNode>
{
  static void mapping(IO &io, DecisionNode &Node)
  {
    io.mapRequired("transformation", Node.Transformation);
    io.mapRequired("children", Node.Children);
    auto Configs = Node.TTuner->getAllConfigs();
    io.mapRequired("configs", Configs);
  }
};

template<>
struct llvm::yaml::MappingTraits<std::unique_ptr<DecisionNode>>
{
  static void mapping(IO &io, std::unique_ptr<DecisionNode> &Ptr)
  {
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


//}
//}

namespace clang {
namespace jit {
inline void writeTree(TransformDecisionTree &Tree, raw_ostream &OS = llvm::outs())
{
  llvm::yaml::Output Out(OS);
  Out << Tree.getRoot();
}

inline void writeTree(TransformDecisionTree &Tree, StringRef File)
{
  std::error_code Err;
  raw_fd_ostream OS(File, Err);
  writeTree(Tree, OS);
}
}
}


#endif // CLANG_DECISIONTREEYAML_H
