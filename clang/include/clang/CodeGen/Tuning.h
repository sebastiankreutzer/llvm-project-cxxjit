//
// Created by sebastian on 30.10.19.
//

#ifndef CLANG_JIT_H
#define CLANG_JIT_H

#include <tuple>
#include <functional>
#include <chrono>

namespace clang {
namespace jit {

template<typename T>
struct tunable_range {
  tunable_range(T Min, T Max) : Min(Min), Max(Max) {};

  T Min;
  T Max;
};

enum class SearchType {
  RANDOM,
  SIMPLEX,
};

struct TunerConfig {
  SearchType TemplateParamSearchType;
  SearchType TransformationParamSearchType;

};

void __clang_jit_enable_tuning(clang::jit::TunerConfig);
void __clang_jit_disable_tuning();

void* finish_tuning(void*);

class TuningScope {
public:
  TuningScope(TunerConfig Cfg) {
    __clang_jit_enable_tuning(Cfg);
  }

  ~TuningScope() {
    __clang_jit_disable_tuning();
  }
};


class TimedTuner {
public:
  TimedTuner(size_t TimeLimit) : TimeLimit(TimeLimit) {

  }



private:
  size_t TimeLimit;
};

}
}


#endif //CLANG_JIT_H
