//
// Created by sebastian on 24.09.19.
//

#include <llvm/Support/raw_ostream.h>
#include "Tuner.h"

using namespace llvm;

namespace clang {
namespace jit {

CachingTuner::CachingTuner()
{

}

ConfigEval CachingTuner::generateNextConfig()
{
  auto Cfg = generateNextConfigInternal();
  auto Eval = getEval(Cfg.first);
  if (Eval) {
    // This can happen if no new config can be found
    // TODO: Return Optional instead?
    do {
      if (attemptRestart()) {
        errs() << "Search is stuck - restarting...\n";
        Cfg = generateNextConfigInternal();
      } else {
        errs() << "No restart possible - stopping search.\n";
        Done = true;
        return *Eval;
      }

    } while (getEval(Cfg.first));

  }
  auto NewEval = ConfigEval(Cfg.first, Cfg.second);
  Evals.push_back(NewEval);
  ConfigMap[Cfg.first] = Evals.size()-1;
  return NewEval;
}

llvm::Optional<ConfigEval> CachingTuner::getEval(const ParamConfig& Config) {
  if (auto It = ConfigMap.find(Config); It != ConfigMap.end()) {
    return Evals[It->second];
  }
  return {};
}

}
}

