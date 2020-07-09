//
// Created by sebastian on 08.07.20.
//

#include "ModifiedSimplexTuner.h"


#include "llvm/ADT/SmallVector.h"

using namespace llvm;

namespace clang {
namespace jit {

ModifiedSimplexTuner::VertexList ModifiedSimplexTuner::createSimplex() const {
  auto perturb = [](auto X0Val, auto Min, auto Max) -> auto {
    assert(X0Val <= Max && Min <= X0Val && "Value out of bounds");
    if (Min == Max)
      return X0Val;
    int Dir = Max - X0Val > X0Val - Min ? 1 : -1;
    auto Delta = ((Max - Min) /
                  4); // TODO: Try out different values, maybe even randomize
    if (Delta == 0) {
      Delta = 1;
    }
    auto Val = X0Val + Dir * Delta;
    return Val;
  };

  VertexList Simplex;
  Simplex.reserve(getNumSimplexPoints());
  auto Default = convertTo<double>(createDefaultConfig(Space));
  auto I = 0;
  for (auto& Dim : Space) {
    auto Cpy = Default;
    Cpy[I]= perturb(Cpy[I], Dim.Min.getAsDouble(), Dim.Max.getAsDouble());
    Simplex.emplace_back(Space, std::move(Cpy));
    I++;
  }
  Simplex.emplace_back(Space, std::move(Default));
  return Simplex;
}

const ConfigEval& ModifiedSimplexTuner::markEvaluated(const ConfigEval& Eval) {
  Evaluated[Eval.Config] = {Eval, {}};
  return Eval;
}

ConfigEval ModifiedSimplexTuner::getNeighborIfAlreadyEvaluated(const ConfigEval& Eval) {
  auto It = Evaluated.find(Eval.Config);
  if (It == Evaluated.end()) {
    return markEvaluated(Eval);
  }
  auto& Entry = It->second;
  auto& Queue = Entry.second;
  if (!Queue.isInitialized())
    Queue.init(Eval.Config);

  // Find the first neighbor that is not already evaluated.
  ParamConfig Neighbor;
  do {
    if (Queue.empty()) {
      // All neighbors evaluated, return original config.
      return Eval;
    }
    Neighbor = Queue.next();
  } while (Evaluated.find(Neighbor) != Evaluated.end());
  return ConfigEval(Neighbor);
}

ConfigEval ModifiedSimplexTuner::generateNextConfig() {
  if (Space.empty()) {
    return ConfigEval();
  }

  if (!EvalQueue.empty())
    return EvalQueue.pop_back_val();

  auto N = Simplex.size();


  switch (State) {
  case INIT: {
    EvalQueue.clear();
    Simplex.clear();

    Simplex = createSimplex();
    for (auto& V : Simplex) {
      EvalQueue.push_back(V.Eval);
      markEvaluated(V.Eval);
    }
    State = START;
    return generateNextConfig();
  }

  case START: {
    assert(EvalQueue.empty() && "Not all vertices evaluated");

    CompareConfigEval Cmp;
    std::sort(Simplex.begin(), Simplex.end(), [&Cmp](Vertex& LHS, Vertex& RHS) -> bool {
      return Cmp(LHS.Eval, RHS.Eval);
    });

    SmallVector<Vector<double>, 4> SimplexVecs;
    SimplexVecs.reserve(Simplex.size());
    for (auto& V : Simplex) {
      SimplexVecs.push_back(V.Vec);
    }

    Centroid = centroid<double>(SimplexVecs.begin(), SimplexVecs.end());

    auto ReflectedVec = Centroid + (Centroid - SimplexVecs.back()) * P.Alpha;

    Reflected = Vertex(Space, ReflectedVec);

    State = EVAL_REFLECTED;
    return markEvaluated(Reflected.Eval);
  }

  case EVAL_REFLECTED: {
    assert(Reflected.Eval.Stats && Reflected.Eval.Stats->Valid() &&
           "Evaluation not finished");

    // Reflection is better than all current vertices -> try expansion.
    if (Reflected.Eval.Stats->betterThan(*Simplex.front().Eval.Stats)) {
      auto ExpandedVec = Centroid + (Reflected.Vec - Centroid) * P.Gamma;
      Expanded = Vertex(Space, ExpandedVec);
      State = EVAL_EXPANDED;
      return markEvaluated(Expanded.Eval);
    }

    // Reflection is better than the second worst vertex -> Replace worst vertex.
    if (Reflected.Eval.Stats->betterThan(*Simplex[N - 2].Eval.Stats)) {
      Simplex[N - 1] = Reflected;
      State = START;
      return generateNextConfig();
    }

    // Reflection is either worst or second worst.
    // Use the better one as base for reflection.
    auto& ContractionBase = Reflected.Eval.Stats->betterThan(*Simplex.back().Eval.Stats) ? Reflected : Simplex.back();

    auto ContractedVec = Centroid + (ContractionBase.Vec - Centroid) * P.Rho;
    Contracted = Vertex(Space, ContractedVec);

    // If the result of the contraction is equal to the base, find a local neighbor.
    // TODO

    State = EVAL_CONTRACTED;
    return markEvaluated(Contracted.Eval);
  }

  case EVAL_EXPANDED: {
    assert(Expanded.Eval.Stats && Expanded.Eval.Stats->Valid() &&
           "Evaluation not finished");
    if (Expanded.Eval.Stats->betterThan(*Reflected.Eval.Stats)) {
      Simplex[N - 1] = Expanded;
    } else {
      Simplex[N - 1] = Reflected;
    }
    State = START;
    return generateNextConfig();
  }

  case EVAL_CONTRACTED: {
    assert(Contracted.Eval.Stats && Contracted.Eval.Stats->Valid() &&
           "Evaluation not finished");
    if (Contracted.Eval.Stats->betterThan(*Simplex.back().Eval.Stats)) {
      Simplex[N - 1] = Contracted;
      State = START;
      return generateNextConfig();
    }
    // Shrink
    auto BestVec = Simplex.front().Vec;
    for (unsigned i = 1; i < N; i++) {
      auto ShrunkVec =
          BestVec + (Simplex[i].Vec - BestVec) * P.Sigma;
      Simplex[i] = Vertex(Space, ShrunkVec);
      EvalQueue.push_back(Simplex[i].Eval);
      markEvaluated(Simplex[i].Eval);
      // TODO: Check if equal and return neighbor
    }
    State = START;
    return generateNextConfig();
  }
  }
  llvm_unreachable("Invalid state reached!");
}

}
}
