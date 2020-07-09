//
// Created by sebastian on 08.07.20.
//

#ifndef CLANG_MODIFIEDSIMPLEXTUNER_H
#define CLANG_MODIFIEDSIMPLEXTUNER_H

#include "Tuner.h"
#include "Search.h"
#include "Math.h"

namespace clang {
namespace jit {

class ModifiedSimplexTuner: public Tuner {

public:

  using EvalList = llvm::SmallVector<ConfigEval, 4>;

  using VectorT  = Vector<double>;
  using VectorList = llvm::SmallVector<VectorT, 4>;

  struct Vertex {
    Vertex() {

    }
    Vertex(SearchSpace& Space, VectorT Vec) : Vec(std::move(Vec)) {
      legalize(Space, this->Vec);
      auto Cfg = createConfig(Space, Vec, ::round);
      assert(Cfg.isLegal());
      Eval = ConfigEval(std::move(Cfg));
    }

    VectorT Vec;
    ConfigEval Eval;
  };

  using VertexList = llvm::SmallVector<Vertex, 4>;

  struct NeighborQueue {
    NeighborQueue() : Initialized(false) {
    }

    void init(const ParamConfig& Config) {
      if (Initialized)
        return;

      assert(Config.Space && "Search space must be defined to find legal neighbors.");

      auto addNeighborInDim = [&](const ParamConfig& C, int Dim, int Offset) {
        ParamConfig Neighbor(C);
        Neighbor[Dim] += Offset;
        if (Neighbor.isLegal()) {
          Neighbors.push_back(std::move(Neighbor));
        }
      };

      for (unsigned I = 0; I < Config.Space->getNumDimensions(); I++) {
        addNeighborInDim(Config, I, -1);
        addNeighborInDim(Config, I, 1);
        for (auto& C : Neighbors) {
          addNeighborInDim(C, I, -1);
          addNeighborInDim(C, I, 1);
        }
      }
      Initialized = true;
    }

    bool isInitialized() const {
      return Initialized;
    }

    bool empty() const {
      return Neighbors.empty();
    }

    ParamConfig next() {
      assert(!empty() && "Queue must not be empty");
      return Neighbors.pop_back_val();
    }

  private:
    bool Initialized;
    llvm::SmallVector<ParamConfig, 2> Neighbors;
  };

  using EvalInfo = std::pair<ConfigEval, NeighborQueue>;

  struct Params {
    float Alpha{1};
    float Gamma{2};
    float Rho{0.5f};
    float Sigma{0.5f};
  };

  enum State {
    INIT, START, EVAL_REFLECTED, EVAL_EXPANDED, EVAL_CONTRACTED
  };

  explicit ModifiedSimplexTuner(SearchSpace& Space)
  : Space(Space) {
      // Dimension = Knobs.
      State = INIT;
  }

  ConfigEval generateNextConfig() override;

  unsigned getNumSimplexPoints() const {
    return Space.getNumDimensions() + 1;
  }

private:
  VertexList createSimplex() const;

  const ConfigEval& markEvaluated(const ConfigEval& Eval);
//  Vertex createVertex(VectorT Vec);
  ConfigEval  getNeighborIfAlreadyEvaluated(const ConfigEval &Eval);


private:
  Params P;
  SearchSpace& Space;

  // unsigned Dimension;
  EvalList EvalQueue;
  VertexList Simplex;

  State State;

  VectorT Centroid;
  Vertex Reflected, Expanded, Contracted;

  unsigned IterCount;

  llvm::DenseMap<HashableConfig, EvalInfo, ParamConfigMapInfo> Evaluated;

};

}
}



#endif // CLANG_MODIFIEDSIMPLEXTUNER_H
