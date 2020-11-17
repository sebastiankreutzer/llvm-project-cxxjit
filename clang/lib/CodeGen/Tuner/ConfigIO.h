#ifndef TUNER_CONFIGIO
#define TUNER_CONFIGIO

#include "llvm/IR/Module.h"
#include "llvm/Support/YAMLTraits.h"

struct SerializableTunerState {
  // Unoptimized module
  std::unique_ptr<llvm::Module> Mod;
};

template <> struct MappingTraits <

#endif
