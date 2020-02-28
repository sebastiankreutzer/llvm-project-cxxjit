//
// Created by sebastian on 10.10.19.
//

#include "SimplexTuner.h"

#include "llvm/ADT/SmallVector.h"

using namespace llvm;

namespace clang {
namespace jit {

//void SimplexTuner::log(StringRef Msg) const {
////  outs() << "Simplex Tuner: " << Msg << "\n";
//}
//
//void SimplexTuner::init() {}
//
//void SimplexTuner::reset(KnobSet Knobs) {
//  this->Knobs = std::move(Knobs);
//  Mapping.remap(&this->Knobs);
//  Initialized = false;
//  State = REFLECT;
//}
//
//// TODO: Requires modification for multiple threads - next config may be
//// requested before last one has been evaluated.
//ConfigEvalRequest SimplexTuner::generateNextConfig() {
//  if (!Initialized) {
//    Initialized = true;
//    auto DefaultCfg = createDefaultConfig(Knobs);
//
//    ToEval = createInitialVertices(Knobs, DefaultCfg);
//    ToEval.push_back(DefaultCfg);
//    IterCount = 0;
//
//    log("Simplex has " + std::to_string(ToEval.size()) + " Vertices");
//  }
//
//  auto N = Simplex.size();
//
//  switch (State) {
//    case REFLECT: {
//      if (!ToEval.empty()) {
//        auto Cfg = ConfigEvalRequest(ToEval.back());
//        ToEval.pop_back();
//        Simplex.push_back(Cfg);
//        return Cfg;
//      }
//      CompareConfigEval Comp;
//      std::sort(Simplex.begin(), Simplex.end(), Comp);
//
//      std::vector<Vector < float>> MappedSimplex;
//      for (unsigned i = 0; i < Simplex.size(); i++) {
////                outs() << "Vertex " << i << ": " << Mapping.map(Simplex[i].Cfg)
////                << "\n"; outs() << "Corresponding config: \n"; KnobState
////                KS(Knobs, Simplex[i].Cfg); KS.dump();
//        MappedSimplex.push_back(Mapping.map(Simplex[i].Cfg));
//      }
//
//      // Check if all at one point
//      auto First = Mapping.legalized(MappedSimplex.front());
//      bool AllEqual = true;
//      for (auto Val : MappedSimplex) {
//        if (!Mapping.legalized(Val).equals(First, 1e-2)) {
//          AllEqual = false;
//          break;
//        }
//      }
//      if (AllEqual) {
////        errs() << "All vertices are equal! Resetting...\n";
//        Simplex.clear();
//        Initialized = false;
//        State = REFLECT;
//        return generateNextConfig();
//      }
//
//      // Worst point is left out
//      Centroid = Mapping.legalized(centroid<float>(MappedSimplex.begin(), MappedSimplex.end() - 1));
//
//      auto ReflectedVec = Mapping.legalized(Centroid + (Centroid - MappedSimplex.back()) * P.Alpha);
//      Reflected = ConfigEvalRequest(Mapping.unmap(ReflectedVec));
//      State = EVAL_REFLECTED;
//      log("Reflection");
//      return Reflected;
//    }
//    case EVAL_REFLECTED: {
//      assert(Reflected.Stats && Reflected.Stats->Valid() &&
//             "Evaluation not finished");
//      if (Reflected.Stats->betterThan(*Simplex.front().Stats)) {
//        auto ExpandedVec =
//            Mapping.legalized(Centroid + (Mapping.map(Reflected.Cfg) - Centroid) * P.Gamma);
//        Expanded = ConfigEvalRequest(Mapping.unmap(ExpandedVec));
//        log("Expansion");
//        State = EVAL_EXPANDED;
//        return Expanded;
//      }
//
//      if (Reflected.Stats->betterThan(*Simplex[N - 2].Stats)) {
//        log("Reflection accepted");
//        Simplex[N - 1] = Reflected;
//        State = REFLECT;
//        return generateNextConfig();
//      }
//
//      auto ContractedVec =
//          Mapping.legalized(Centroid + (Mapping.map(Simplex.back().Cfg) - Centroid) * P.Rho);
//      Contracted = ConfigEvalRequest(Mapping.unmap(ContractedVec));
//      State = EVAL_CONTRACTED;
//      log("Contraction");
//      return Contracted;
//    }
//
//    case EVAL_EXPANDED: {
//      assert(Expanded.Stats && Expanded.Stats->Valid() &&
//             "Evaluation not finished");
//      if (Expanded.Stats->betterThan(*Reflected.Stats)) {
//        Simplex[N - 1] = Expanded;
//        log("Expansion accepted");
//        State = REFLECT;
//        return generateNextConfig();
//      }
//      Simplex[N - 1] = Reflected;
//      log("Reflection accepted");
//      State = REFLECT;
//      return generateNextConfig();
//    }
//
//    case EVAL_CONTRACTED: {
//      assert(Contracted.Stats && Contracted.Stats->Valid() &&
//             "Evaluation not finished");
//      if (Contracted.Stats->betterThan(*Simplex.back().Stats)) {
//        log("Contraction accepted");
//        Simplex[N - 1] = Contracted;
//        State = REFLECT;
//        return generateNextConfig();
//      }
//      // Shrink
//      log("Shrinking");
//      auto BestVec = Mapping.map(Simplex.front().Cfg);
//      for (unsigned i = 1; i < N; i++) {
//        auto ShrunkVec =
//            BestVec + (Mapping.map(Simplex[i].Cfg) - BestVec) * P.Sigma;
//        ToEval.push_back(Mapping.unmap(ShrunkVec));
//      }
//      Simplex.erase(Simplex.begin() + 1, Simplex.end());
//      State = REFLECT;
//      return generateNextConfig();
//    }
//  }
//  llvm_unreachable("Invalid state reached!");
//}

SimplexTuner::VertexList SimplexTuner::createSimplex() const {
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

}

ConfigEval SimplexTuner::generateNextConfig() {
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

      auto Centroid = centroid<double>(SimplexVecs.begin(), SimplexVecs.end());

      auto ReflectedVec = Centroid + (Centroid - SimplexVecs.back()) * P.Alpha;

      Reflected = Vertex(Space, ReflectedVec);

      State = EVAL_REFLECTED;
      return Reflected.Eval;
    }

    case EVAL_REFLECTED: {
      assert(Reflected.Eval.Stats && Reflected.Eval.Stats->Valid() &&
             "Evaluation not finished");
      if (Reflected.Eval.Stats->betterThan(*Simplex.front().Eval.Stats)) {
        auto ExpandedVec = Centroid + (Reflected.Vec - Centroid) * P.Gamma;
        Expanded = Vertex(Space, ExpandedVec);
        State = EVAL_EXPANDED;
        return Expanded.Eval;
      }

      if (Reflected.Eval.Stats->betterThan(*Simplex[N - 2].Eval.Stats)) {
        Simplex[N - 1] = Reflected;
        State = START;
        return generateNextConfig();
      }

      auto ContractedVec = Centroid + (Simplex.back().Vec - Centroid) * P.Rho;
      Contracted = Vertex(Space, ContractedVec);
      State = EVAL_CONTRACTED;
      return Contracted.Eval;
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
      }
      State = START;
      return generateNextConfig();
    }
  }
  llvm_unreachable("Invalid state reached!");
}

}
}