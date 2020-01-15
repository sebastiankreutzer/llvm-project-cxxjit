//
// Created by sebastian on 10.10.19.
//

#include "SimplexTuner.h"

namespace clang {
namespace jit {

void SimplexTuner::log(StringRef Msg) const {
//  outs() << "Simplex Tuner: " << Msg << "\n";
}

void SimplexTuner::init() {}

void SimplexTuner::reset(KnobSet Knobs) {
  this->Knobs = std::move(Knobs);
  Mapping.remap(&this->Knobs);
  Initialized = false;
  State = REFLECT;
}

// TODO: Requires modification for multiple threads - next config may be
// requested before last one has been evaluated.
ConfigEvalRequest SimplexTuner::generateNextConfig() {
  if (!Initialized) {
    Initialized = true;
    auto DefaultCfg = createDefaultConfig(Knobs);

    ToEval = createInitialVertices(Knobs, DefaultCfg);
    ToEval.push_back(DefaultCfg);
    IterCount = 0;

    log("Simplex has " + std::to_string(ToEval.size()) + " Vertices");
  }

  auto N = Simplex.size();

  switch (State) {
    case REFLECT: {
      if (!ToEval.empty()) {
        auto Cfg = ConfigEvalRequest(ToEval.back());
        ToEval.pop_back();
        Simplex.push_back(Cfg);
        return Cfg;
      }
      CompareConfigEval Comp;
      std::sort(Simplex.begin(), Simplex.end(), Comp);

      std::vector<Vector < float>> MappedSimplex;
      for (unsigned i = 0; i < Simplex.size(); i++) {
//                outs() << "Vertex " << i << ": " << Mapping.map(Simplex[i].Cfg)
//                << "\n"; outs() << "Corresponding config: \n"; KnobState
//                KS(Knobs, Simplex[i].Cfg); KS.dump();
        MappedSimplex.push_back(Mapping.map(Simplex[i].Cfg));
      }

      // Check if all at one point
      auto First = Mapping.legalized(MappedSimplex.front());
      bool AllEqual = true;
      for (auto Val : MappedSimplex) {
        if (!Mapping.legalized(Val).equals(First, 1e-2)) {
          AllEqual = false;
          break;
        }
      }
      if (AllEqual) {
//        errs() << "All vertices are equal! Resetting...\n";
        Simplex.clear();
        Initialized = false;
        State = REFLECT;
        return generateNextConfig();
      }

      // Worst point is left out
      Centroid = Mapping.legalized(centroid<float>(MappedSimplex.begin(), MappedSimplex.end() - 1));

      auto ReflectedVec = Mapping.legalized(Centroid + (Centroid - MappedSimplex.back()) * P.Alpha);
      Reflected = ConfigEvalRequest(Mapping.unmap(ReflectedVec));
      State = EVAL_REFLECTED;
      log("Reflection");
      return Reflected;
    }
    case EVAL_REFLECTED: {
      assert(Reflected.Stats && Reflected.Stats->Valid() &&
             "Evaluation not finished");
      if (Reflected.Stats->betterThan(*Simplex.front().Stats)) {
        auto ExpandedVec =
            Mapping.legalized(Centroid + (Mapping.map(Reflected.Cfg) - Centroid) * P.Gamma);
        Expanded = ConfigEvalRequest(Mapping.unmap(ExpandedVec));
        log("Expansion");
        State = EVAL_EXPANDED;
        return Expanded;
      }

      if (Reflected.Stats->betterThan(*Simplex[N - 2].Stats)) {
        log("Reflection accepted");
        Simplex[N - 1] = Reflected;
        State = REFLECT;
        return generateNextConfig();
      }

      auto ContractedVec =
          Mapping.legalized(Centroid + (Mapping.map(Simplex.back().Cfg) - Centroid) * P.Rho);
      Contracted = ConfigEvalRequest(Mapping.unmap(ContractedVec));
      State = EVAL_CONTRACTED;
      log("Contraction");
      return Contracted;
    }

    case EVAL_EXPANDED: {
      assert(Expanded.Stats && Expanded.Stats->Valid() &&
             "Evaluation not finished");
      if (Expanded.Stats->betterThan(*Reflected.Stats)) {
        Simplex[N - 1] = Expanded;
        log("Expansion accepted");
        State = REFLECT;
        return generateNextConfig();
      }
      Simplex[N - 1] = Reflected;
      log("Reflection accepted");
      State = REFLECT;
      return generateNextConfig();
    }

    case EVAL_CONTRACTED: {
      assert(Contracted.Stats && Contracted.Stats->Valid() &&
             "Evaluation not finished");
      if (Contracted.Stats->betterThan(*Simplex.back().Stats)) {
        log("Contraction accepted");
        Simplex[N - 1] = Contracted;
        State = REFLECT;
        return generateNextConfig();
      }
      // Shrink
      log("Shrinking");
      auto BestVec = Mapping.map(Simplex.front().Cfg);
      for (unsigned i = 1; i < N; i++) {
        auto ShrunkVec =
            BestVec + (Mapping.map(Simplex[i].Cfg) - BestVec) * P.Sigma;
        ToEval.push_back(Mapping.unmap(ShrunkVec));
      }
      Simplex.erase(Simplex.begin() + 1, Simplex.end());
      State = REFLECT;
      return generateNextConfig();
    }
  }
  llvm_unreachable("Invalid state reached!");
}

}
}