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

using TaggedConfig = std::pair<ParamConfig, std::string>;

class CachedModifiedSimplexTuner: public CachingTuner {

public:
  using EvalList = llvm::SmallVector<TaggedConfig, 4>;

  using Scalar = double;
  using ParamVector  = Vector<Scalar>;
  using VectorList = llvm::SmallVector<ParamVector, 4>;

//  struct Vertex {
//    Vertex() {
//
//    }
//
////    Vertex(const SearchSpace& Space, VectorT Vec, ConfigEval Eval) : Vec(std::move(Vec)), Eval(std::move(Eval)) {
////      legalize()
////    }
//
//    Vertex(const SearchSpace& Space, ParamVector Vec, std::string Op = "") : Vec(std::move(Vec)) {
//      legalize(Space, this->Vec);
//      auto Cfg = createConfig(Space, this->Vec, ::round);
//      assert(Cfg.isLegal());
//      this->Cfg = {Cfg, Op};
//    }
//
//    ParamVector Vec;
//    TaggedConfig Cfg;
//  };

  using ConfigList = llvm::SmallVector<TaggedConfig, 4>;

  struct Params {
    float Alpha{1};
    float Gamma{2};
    float Rho{0.5f};
    float Sigma{0.5f};
  };

  enum State {
    INIT, START, EVAL_REFLECTED, EVAL_EXPANDED, EVAL_CONTRACTED
  };

  explicit CachedModifiedSimplexTuner(SearchSpace& Space)
  : Space(Space), RNE(TunerRNE(util::genSeed())) {
      // Dimension = Knobs.
      State = INIT;
  }

  unsigned getNumSimplexPoints() const {
    return Space.getNumDimensions() + 1;
  }

private:
  ConfigList createSimplex() const;
  ConfigList createSimplex2(ParamConfig BaseConfig) const;


  llvm::Optional<TaggedConfig> getFreshNeighbor(const ParamConfig&, std::string Op);

  TaggedConfig generateNextConfigInternal() override;

  TaggedConfig getNextVertex();

  TaggedConfig getLegalizedConfig(ParamVector Vec, std::string Op) const;

  bool attemptRestart() override;


private:
  Params P;
  SearchSpace& Space;

  // unsigned Dimension;
  EvalList EvalQueue;
  ConfigList Simplex;

  State State;

  ParamVector Centroid;
  TaggedConfig Reflected, Expanded, Contracted;

  TunerRNE RNE;

  //unsigned IterCount;

};

}
}



#endif // CLANG_MODIFIEDSIMPLEXTUNER_H
