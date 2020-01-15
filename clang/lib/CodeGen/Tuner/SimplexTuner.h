//
// Created by sebastian on 10.10.19.
//

#ifndef CLANG_SIMPLEXTUNER_H
#define CLANG_SIMPLEXTUNER_H

#include "ConfigMath.h"
#include "Tuner.h"

namespace clang {
namespace jit {

struct GenInitialVerticesFn : public KnobSetFn {

  explicit GenInitialVerticesFn(KnobConfig X0) : X0(X0) {}

  // Perturbs int, should work for boolean knobs
  int perturbInt(int X0Val, int Min, int Max) {
    assert(X0Val <= Max && Min <= X0Val && "Value out of bounds");
    if (Min == Max)
      return X0Val;
    int Dir = Max - X0Val > X0Val - Min ? 1 : -1;
    int Delta = ((Max - Min) /
                 4); // TODO: Try out different values, maybe even randomize
    if (Delta == 0) {
      Delta = 1;
    }
    int Val = X0Val + Dir * Delta;
    return Val;
  }

  void operator()(IntKnob &K) override {
    KnobConfig X(X0);
    int Val = perturbInt(K.getVal(X0), K.min(), K.max());
    K.setVal(X, Val);
    Vertices.push_back(X);
  }

  void operator()(LoopKnob &K) override {

//    outs() << "Generating vertices for knob: " << K.getName() << ":\n";
    for (unsigned i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
      auto Param = (LoopTransformConfig::Parameter) i;
      if (!K.isTunable(Param))
        continue;

      KnobConfig X(X0);
      auto LoopCfg = K.getVal(X0);

      auto Min = K.getMin(Param);
      auto Max = K.getMax(Param);
      auto Val = perturbInt(LoopCfg.Vals[i], Min, Max);
//      outs() << "Attribute " << Param << " is tunable: min=" << Min << ", max=" << Max << ", val=" << Val << "\n";
      LoopCfg.Vals[i] = (unsigned) Val;
      K.setVal(X, LoopCfg);
      Vertices.push_back(X);
    }
  }

  std::vector<KnobConfig> Vertices;
  KnobConfig X0;
};

struct ComputeCentroidFn : public KnobSetFn {
  explicit ComputeCentroidFn(std::vector<KnobConfig> &Configs)
      : Configs(Configs) {}

  void operator()(IntKnob &K) override {
    int Val = 0;
    for (auto &Cfg : Configs) {
      Val += K.getVal(Cfg);
    }
    K.setVal(Centroid, Val / Configs.size());
  }

  void operator()(LoopKnob &K) override {
    LoopTransformConfig Val;

    for (auto i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
      int ParamVal = 0;
      for (auto &Cfg : Configs) {
        ParamVal += K.getVal(Cfg).Vals[i];
      }
      Val.Vals[i] = ParamVal / Configs.size();
    }
    K.setVal(Centroid, Val);
  }

  std::vector<KnobConfig> &Configs;
  KnobConfig Centroid;
};

inline std::vector<KnobConfig> createInitialVertices(KnobSet &Knobs,
                                                     KnobConfig X0) {
  GenInitialVerticesFn Fn(std::move(X0));
  apply(Fn, Knobs);
  return Fn.Vertices;
}

inline KnobConfig computeCentroid(KnobSet &Knobs,
                                  std::vector<KnobConfig> &Configs) {
  ComputeCentroidFn Fn(Configs);
  apply(Fn, Knobs);
  return Fn.Centroid;
}

class SimplexTuner : public Tuner {
public:
  struct Params {
    float Alpha{1};
    float Gamma{2};
    float Rho{0.5f};
    float Sigma{0.5f};
  };

  enum State {
    REFLECT, EVAL_REFLECTED, EVAL_EXPANDED, EVAL_CONTRACTED
  };

  explicit SimplexTuner(KnobSet Knobs)
      : Knobs(std::move(Knobs)), Mapping(&this->Knobs) {
    // Dimension = Knobs.
    State = REFLECT;
  }

  void init();

  void reset(KnobSet Knobs) override;

  ConfigEvalRequest generateNextConfig() override;

private:
  void log(StringRef Msg) const;

private:
  Params P;
  KnobSet Knobs;
  VectorMapping<float> Mapping;
  // unsigned Dimension;
  std::vector<ConfigEvalRequest> Simplex;
  std::vector<KnobConfig> ToEval;

  State State;

  Vector<float> Centroid;
  ConfigEvalRequest Reflected, Expanded, Contracted;

  bool Initialized{false};
  unsigned IterCount;
};

}
}

#endif // CLANG_SIMPLEXTUNER_H
