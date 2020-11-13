//
// Created by sebastian on 08.07.20.
//

#include "ModifiedSimplexTunerCached.h"


#include "llvm/ADT/SmallVector.h"

#include "Debug.h"

// Enables always selecting a neighboring point, if the current config is already evaluated.
// By default, only shrink and contract use this.
//#define ALWAYS_SEARCH_NEIGHBORS

using namespace llvm;

namespace clang {
namespace jit {

CachedModifiedSimplexTuner::ConfigList CachedModifiedSimplexTuner::createSimplex() const {
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

  ConfigList Simplex;
  Simplex.reserve(getNumSimplexPoints());
  auto DefaultCfg = createDefaultConfig(Space);
  auto Default = convertTo<Scalar>(DefaultCfg);
  auto I = 0;
  for (auto& Dim : Space) {
    auto Cpy = Default;
    Cpy[I]= perturb(Cpy[I], Dim.Min.getAsDouble(), Dim.Max.getAsDouble());
    Simplex.push_back(getLegalizedConfig(Cpy, "INIT"));
    I++;
  }
  Simplex.emplace_back(DefaultCfg, "INIT");
  return Simplex;
}

CachedModifiedSimplexTuner::ConfigList CachedModifiedSimplexTuner::createSimplex2(ParamConfig BaseConfig) const {
  auto perturb = [](auto X0Val, auto Min, auto Max) -> auto {
    assert(X0Val <= Max && Min <= X0Val && "Value out of bounds");
    return Max;
  };

  ConfigList Simplex;
  Simplex.reserve(getNumSimplexPoints());
  auto Default = convertTo<Scalar>(BaseConfig);
  auto I = 0;
  for (auto& Dim : Space) {
    auto Cpy = Default;
    Cpy[I]= perturb(Cpy[I], Dim.Min.getAsDouble(), Dim.Max.getAsDouble());
    Simplex.push_back(getLegalizedConfig(Cpy, "INIT"));
    I++;
  }
  Simplex.emplace_back(BaseConfig, "INIT");
  return Simplex;
}


Optional<TaggedConfig> CachedModifiedSimplexTuner::getFreshNeighbor(const ParamConfig& Config, std::string Op) {

  auto addNeighborInDim = [&](const ParamConfig& C, int Dim, int Offset, llvm::SmallVectorImpl<ParamConfig>& ConfigList) {
    ParamConfig Neighbor(C);
    Neighbor[Dim] += Offset;
    if (Neighbor.isLegal()) {
      ConfigList.push_back(std::move(Neighbor));
    }
  };

  // Search neighbors iteratively in each dimension.

  llvm::SmallVector<ParamConfig, 8> Neighbors;
  for (unsigned I = 0; I < Space.getNumDimensions(); I++) {
    unsigned NumNeighbors = Neighbors.size();
    for (unsigned J = 0; J < NumNeighbors; J++) {
      auto& C = Neighbors[J];
      addNeighborInDim(C, I, -1, Neighbors);
      addNeighborInDim(C, I, 1, Neighbors);
    }
    addNeighborInDim(Config, I, -1, Neighbors);
    addNeighborInDim(Config, I, 1, Neighbors);

    // Check if one of the neighbors is not evaluated yet.
    for (auto& Neighbor : Neighbors) {
      if (!isEvaluated(Neighbor)) {
//        outs()  << "Replacing config: ";
//        Config.dump();
//        outs() << " with ";
//        Neighbor.dump();
        return {{Neighbor, Op}};
      }
    }
  }
  return {};
}


TaggedConfig CachedModifiedSimplexTuner::generateNextConfigInternal()  {
  if (Space.empty()) {
    return {ParamConfig(), "EMPTY"};
  }
  TaggedConfig Config;
  unsigned Tries = 0;
  do {
    Config = getNextVertex();
    Tries++;
  } while (isEvaluated(Config.first) && Tries < 10);
  // TODO: better way to handle the case if no new config available
  return Config;
}

TaggedConfig CachedModifiedSimplexTuner::getNextVertex() {
  if (!EvalQueue.empty())
    return EvalQueue.pop_back_val();

  auto N = Simplex.size();

  auto getSimplexStats = [&](unsigned I) {
    auto Eval = getEval(Simplex.front().first);
    assert(Eval && "Simplex point not evaluated");
    return Eval->Stats;
  };


  switch (State) {
    case INIT: {
      EvalQueue.clear();
      Simplex.clear();

      auto Base = createMinConfig(Space);
      if (NumRestarts == 1) {
        Base = createDefaultConfig(Space);
        JIT_INFO(outs() << "Search restarted with minimum base config\n");
      } else if (NumRestarts > 1) {
        Base = createRandomConfig(RNE, Space);
        JIT_INFO(outs() << "Search restarted with random base config\n");
      }
      Simplex = createSimplex2(Base);

      for (auto& V : Simplex) {
        EvalQueue.push_back(V);
      }
      State = START;
      return getNextVertex();
    }

    case START: {
      assert(EvalQueue.empty() && "Not all vertices evaluated");

      CompareConfigEval Cmp;
      std::sort(Simplex.begin(), Simplex.end(), [&Cmp, this](TaggedConfig & LHS, TaggedConfig & RHS) -> bool {
        auto LHSEval = getEval(LHS.first);
        auto RHSEval = getEval(RHS.first);
        assert(LHSEval && RHSEval);
        return Cmp(LHSEval.getValue(), RHSEval.getValue());
      });

      SmallVector<Vector<Scalar>, 4> SimplexVecs;
      SimplexVecs.reserve(Simplex.size());
      for (auto& V : Simplex) {
        SimplexVecs.push_back(convertTo<Scalar>(V.first));
      }

      Centroid = centroid<Scalar>(SimplexVecs.begin(), SimplexVecs.end()-1);

      auto ReflectedVec = Centroid + (Centroid - SimplexVecs.back()) * P.Alpha;

      Reflected = getLegalizedConfig(ReflectedVec, "REFLECT");

#ifdef ALWAYS_SEARCH_NEIGHBORS
      if (isEvaluated(Reflected.first)) {
        auto Neighbor = getFreshNeighbor(Reflected.first, "REFLECT (ADJUSTED)");
        if (Neighbor)
          Reflected = *Neighbor;
      }
#endif

      State = EVAL_REFLECTED;
      return Reflected;
    }

    case EVAL_REFLECTED: {
      auto ReflectedEval = getEval(Reflected.first);
      assert(ReflectedEval);
      auto ReflectedStats = ReflectedEval->Stats;
      assert(ReflectedStats && ReflectedStats->valid() &&
             "Evaluation not finished");

      // Reflection is better than all current vertices -> try expansion.
      if (ReflectedStats->betterThan(*getSimplexStats(0))) {
        auto ExpandedVec = Centroid + (convertTo<Scalar>(Reflected.first) - Centroid) * P.Gamma;
        Expanded = getLegalizedConfig(ExpandedVec, "EXPAND");


#ifdef ALWAYS_SEARCH_NEIGHBORS
        if (isEvaluated(Expanded.first)) {
          auto Neighbor = getFreshNeighbor(Expanded.first, "EXPAND (ADJUSTED)");
          if (Neighbor)
            Expanded = *Neighbor;
        }
#endif

        State = EVAL_EXPANDED;
        return Expanded;
      }

      // Reflection is better than the second worst vertex -> Replace worst vertex.
      if (ReflectedStats->betterThan(*getSimplexStats(N-2))) {
        Simplex[N - 1] = Reflected;
        State = START;
        return getNextVertex();
      }

      // Reflection is either worst or second worst.
      // Use the better one as base for reflection.
      auto& ContractionBase = ReflectedStats->betterThan(*getSimplexStats(N-1)) ? Reflected : Simplex.back();

      auto ContractedVec = Centroid + (convertTo<Scalar>(ContractionBase.first) - Centroid) * P.Rho;
      Contracted = getLegalizedConfig(ContractedVec, "CONTRACT");

      // If the result of the contraction is equal to the base, find a local neighbor.
      if (isEvaluated(Contracted.first)) {
        auto Neighbor = getFreshNeighbor(Simplex.front().first, "CONTRACT (ADJUSTED)");
        if (Neighbor)
          Contracted = *Neighbor;
      }

      // TODO: Restart if no neighbor remaining?

      State = EVAL_CONTRACTED;
      return Contracted;
    }

    case EVAL_EXPANDED: {
      auto ExpandedEval = getEval(Expanded.first);
      assert(ExpandedEval);
      auto ExpandedStats = ExpandedEval->Stats;
      assert(ExpandedStats && ExpandedStats->valid() &&
             "Evaluation not finished");

      auto ReflectedEval = getEval(Reflected.first);
      assert(ReflectedEval);
      auto ReflectedStats = ReflectedEval->Stats;
      assert(ReflectedStats && ReflectedStats->valid() &&
             "Evaluation not finished");

      if (ExpandedStats->betterThan(*ReflectedStats)) {
        Simplex[N - 1] = Expanded;
      } else {
        Simplex[N - 1] = Reflected;
      }
      State = START;
      return getNextVertex();
    }

    case EVAL_CONTRACTED: {

      auto ContractedEval = getEval(Contracted.first);
      assert(ContractedEval);
      auto ContractedStats = ContractedEval->Stats;
      assert(ContractedStats && ContractedStats->valid() &&
             "Evaluation not finished");

      assert(ContractedStats && ContractedStats->valid() &&
             "Evaluation not finished");
      if (ContractedStats->betterThan(*getSimplexStats(N-1))) {
        Simplex[N - 1] = Contracted;
        State = START;
        return getNextVertex();
      }
      // Shrink
      auto BestVec = convertTo<Scalar>(Simplex.front().first);
      for (unsigned i = 1; i < N; i++) {
        auto ShrunkVec =
                BestVec + (convertTo<Scalar>(Simplex[i].first) - BestVec) * P.Sigma;
        Simplex[i] = getLegalizedConfig(ShrunkVec, "SHRINK");

        if (isEvaluated(Simplex[i].first)) {
          auto Neighbor = getFreshNeighbor(Simplex.front().first, "SHRINK (ADJUSTED)");
          if (Neighbor)
            Simplex[i] = *Neighbor;
        }

        EvalQueue.push_back(Simplex[i]);
      }
      State = START;
      return getNextVertex();
    }
  }
  llvm_unreachable("Invalid state reached!");
}

TaggedConfig CachedModifiedSimplexTuner::getLegalizedConfig(CachedModifiedSimplexTuner::ParamVector Vec, std::string Op) const
{
  legalize(Space, Vec);
  auto Cfg = createConfig(Space, Vec, ::round);
  assert(Cfg.isLegal());
  return {Cfg, Op};
}

bool CachedModifiedSimplexTuner::attemptRestart()
{
  if (NumRestarts >= 3) {
    return false;
  }
  NumRestarts++;
  State = INIT;
  return true;
}

}
}

#undef ALWAYS_SEARCH_NEIGHBORS
