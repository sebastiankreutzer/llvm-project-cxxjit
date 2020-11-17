//
// Created by sebastian on 10.10.19.
//

#ifndef CLANG_SIMPLEXTUNER_H
#define CLANG_SIMPLEXTUNER_H

//#include "ConfigMath.h"
#include "Tuner.h"
#include "Search.h"
#include "Math.h"

namespace clang {
namespace jit {


class SimplexTuner : public Tuner {
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

  struct Params {
    float Alpha{1};
    float Gamma{2};
    float Rho{0.5f};
    float Sigma{0.5f};
  };

  enum State {
    INIT, START, EVAL_REFLECTED, EVAL_EXPANDED, EVAL_CONTRACTED
  };

  explicit SimplexTuner(SearchSpace& Space)
      : Space(Space) {
    State = INIT;
    assert(Space.getNumDimensions() == Space.getNumTunableDimensions() && "All dimensions must have multiple possible values");
  }

  ConfigEval generateNextConfig() override;

  unsigned getNumSimplexPoints() const {
    return Space.getNumDimensions() + 1;
  }

private:
  VertexList createSimplex() const;


//  Vertex createVertex(VectorT Vec);



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
};

}
}

#endif // CLANG_SIMPLEXTUNER_H
