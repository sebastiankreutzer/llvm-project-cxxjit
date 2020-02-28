//
// Created by sebastian on 28.01.20.
//

#include <llvm/Support/FormatVariadic.h>
#include "Search.h"

using namespace llvm;

namespace clang {
namespace jit {

char WrongParamType::ID;



raw_ostream& operator<<(raw_ostream& OS, const ParamVal& Val) {
  switch(Val.Type) {
    case ParamType::INT:
      return OS << cantFail(Val.getIntVal());
    case ParamType::FP:
      return OS << formatv("{0:f2}", cantFail(Val.getFPVal()));
  }
  llvm_unreachable("missed case");
}


void ParamConfig::dump(llvm::raw_ostream &OS) const {
  if (isEmpty()) {
    OS << "Empty configuration.\n";
    return;
  }
  if (!Space) {
    OS << "Search space undefined, but contains values:\n";
    for (auto& Val : Values) {
      OS << "Unknown parameter : " << Val << "\n";
    }
  } else {
    if (Space->getNumDimensions() != Values.size()) {
      OS << "WARNING: Search space has " << Space->getNumDimensions() << " dimensions, but the configurations has " << Values.size() << " entries!\n";
    }

    unsigned I = 0;
    for (auto& Param : *Space) {
      if (I >= Values.size())
        break;
      OS << Param.Name << " [ " << Param.Min << "; " << Param.Max << " ] : " << Values[I] << "\n";
      I++;
    }

  }

}

}
}