//
// Created by sebastian on 24.09.19.
//

#ifndef CLANG_TUNER_H
#define CLANG_TUNER_H

#include <random>

#include "Search.h"
#include "Util.h"

namespace clang {
namespace jit {

using TunerRNE = std::mt19937_64;

struct TimingStats {
  unsigned N{0};
  double Mean{0};
  double Variance{0};
  double SD{0};

  TimingStats() = default;

  TimingStats(unsigned N, double Mean, double Variance)
      : N(N), Mean(Mean), Variance(Variance) {
    SD = std::sqrt(Variance);
  }

  bool Valid() const { return N > 0 && Mean > 0 && Variance >= 0; }

  bool betterThan(const TimingStats &Other) {
    // TODO: Do t-test instead
    if (!Valid())
      return false;
    if (!Other.Valid())
      return true;
    return Mean < Other.Mean;
  }

  bool operator<(const TimingStats &Other) { return betterThan(Other); }

  double getTotal() { return N * Mean; }

  double getRSD() { return SD / Mean; }

  double getStdErrOfMean() { return SD / std::sqrt(N); }

  double getRelativeStdErr() { return getStdErrOfMean() / Mean; }
};

using SharedEvalStats = std::shared_ptr<TimingStats>;

#if 0
class ConfigEvalRequest {
public:
  ConfigEvalRequest() : Stats(std::make_shared<TimingStats>()){
  };

  explicit ConfigEvalRequest(KnobConfig Cfg)
      : Cfg(std::move(Cfg)), Stats(std::make_shared<TimingStats>()) {}

  KnobConfig Cfg;
  SharedEvalStats Stats;
};

struct CompareConfigEval {
  bool operator()(ConfigEvalRequest &A, ConfigEvalRequest &B) {
    if (!A.Stats || !A.Stats->Valid())
      return false;
    if (!B.Stats || !B.Stats->Valid())
      return true;
    return A.Stats->betterThan(*B.Stats);
  }
};

class Tuner {
public:
  virtual ~Tuner() {};

  virtual void reset(KnobSet Knobs) = 0;

  virtual ConfigEvalRequest generateNextConfig() = 0;
};
#endif

struct ConfigEval {
  ConfigEval() : Stats(std::make_shared<TimingStats>()) {

  }

  explicit ConfigEval(SearchSpace* Space) : Config(*Space), Stats(std::make_shared<TimingStats>()) {

  }

  explicit ConfigEval(ParamConfig Config) : Config(std::move(Config)), Stats(std::make_shared<TimingStats>()) {

  }

  bool isConfigEmpty() const {
    return Config.isEmpty();
  }

  ParamConfig Config;
  SharedEvalStats Stats;

};

struct CompareConfigEval {
  bool operator()(ConfigEval &A, ConfigEval &B) {
    if (!A.Stats || !A.Stats->Valid())
      return false;
    if (!B.Stats || !B.Stats->Valid())
      return true;
    return A.Stats->betterThan(*B.Stats);
  }
};

class Tuner {
public:
  virtual ~Tuner() {}

  virtual ConfigEval generateNextConfig() = 0;
};

//struct GenDefaultConfigFn : public KnobSetFn {
//
//  void operator()(IntKnob &K) override { K.setVal(Cfg, K.getDefault()); }
//
//  void operator()(LoopKnob &K) override { K.setVal(Cfg, K.getDefault()); }
//
//  KnobConfig Cfg;
//};
//
//template<typename RNETy>
//struct GenRandomConfigFn : public KnobSetFn {
//
//  explicit GenRandomConfigFn(RNETy &RNE) : RNE(RNE) {};
//
//  void operator()(IntKnob &K) override {
//    std::uniform_int_distribution<int> dist(K.min(), K.max());
//    auto Val = dist(RNE);
//    K.setVal(Cfg, Val);
//  }
//
//  void operator()(LoopKnob &K) override {
//    auto LCfg = createRandomLoopConfig(K, RNE);
//    K.setVal(Cfg, LCfg);
//  }
//
//  RNETy &RNE;
//  KnobConfig Cfg;
//};
//
//inline void setEnableLoopTransform(KnobConfig &Cfg, bool Enable) {
//  for (auto &It : Cfg.LoopCfg) {
//    It.second.DisableLoopTransform = !Enable;
//  }
//}
//
//template<typename RNETy>
//KnobConfig createRandomConfig(RNETy &RNE, KnobSet &Set) {
//  GenRandomConfigFn<RNETy> Fn(RNE);
//  apply(Fn, Set);
//  return Fn.Cfg;
//}
//
//inline KnobConfig createDefaultConfig(KnobSet &Set) {
//  GenDefaultConfigFn Fn;
//  apply(Fn, Set);
//  return Fn.Cfg;
//}

class RandomTuner : public Tuner {
public:
  explicit RandomTuner(SearchSpace& Space)
      : Space(Space), RNE(TunerRNE(util::genSeed())) {}

//  void reset(KnobSet Knobs) override { this->Knobs = std::move(Knobs); }

  ConfigEval generateNextConfig() override {
    //    for (auto It : Knobs.IntKnobs)
    //      outs() << It.first << ": " << It.second->getName() << "\n";
    //    for (auto It : Knobs.LoopKnobs)
    //      outs() << It.first << ": " << It.second->getName() << "\n";
    return ConfigEval(createRandomConfig(RNE, Space));
  }

private:
//  KnobSet Knobs;
  SearchSpace& Space;
  TunerRNE RNE;
};

class BilevelTuner {
public:
  explicit BilevelTuner(Tuner &L1Tuner) : L1Tuner(L1Tuner) {};

  virtual bool updatePartialConfig() = 0;

  ParamConfig getPartialConfig() const { return PartialConfig; }

  ConfigEval generateNextL2Config(Tuner &L2Tuner) {
    auto EvalRequest = L2Tuner.generateNextConfig();
    CurrentL2Configs.push_back(EvalRequest);
    return EvalRequest;
  }

protected:
  void changeL1Config(ParamConfig L1Cfg) {
    PartialConfig = std::move(L1Cfg);
    CurrentL2Configs.clear();
  }

protected:
  Tuner &L1Tuner;

  ParamConfig PartialConfig;

  llvm::SmallVector<ConfigEval, 8> CurrentL2Configs;
};

class SimpleBilevelTuner : public BilevelTuner {
public:
  explicit SimpleBilevelTuner(Tuner &L1Tuner) : BilevelTuner(L1Tuner) {};

  bool updatePartialConfig() override {
    if (CurrentL2Configs.size() >= 8) {
      changeL1Config(L1Tuner.generateNextConfig().Config);
    }
    return false;
  }
};


}
}

#endif // CLANG_TUNER_H
