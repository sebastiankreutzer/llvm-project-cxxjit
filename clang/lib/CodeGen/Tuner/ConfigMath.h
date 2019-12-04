//
// Created by sebastian on 10.10.19.
//

#ifndef CLANG_CONFIGMATH_H
#define CLANG_CONFIGMATH_H

#include "llvm/Support/FormatVariadic.h"
#include <cassert>
#include <vector>

#include "KnobSet.h"

namespace tuner {

// TODO: Inefficient vector implementation, but will do for now
template <typename T> class Vector {
  using VecT = Vector<T>;

public:
  explicit Vector(size_t N) : Size(N), Vals(N) {
    for (size_t i = 0; i < N; i++) {
      Vals[i] = 0;
    }
  }

  Vector() : Vector(1){};

  Vector(const Vector &Other) = default;

  Vector<T> &operator=(const Vector &Other) = default;

  T &operator[](size_t I) { return Vals[I]; }

  const T &operator[](size_t I) const { return Vals[I]; }

  void operator*=(T K) {
    for (size_t i = 0; i < Size; i++) {
      Vals[i] *= K;
    }
  }

  VecT operator*(T K) {
    VecT Res(*this);
    Res *= K;
    return Res;
  }

  void operator/=(T K) {
    for (size_t i = 0; i < Size; i++) {
      Vals[i] /= K;
    }
  }

  VecT operator/(T K) {
    VecT Res(*this);
    Res /= K;
    return Res;
  }

  void operator+=(const VecT &Other) {
    assert(Size == Other.Size);
    for (size_t i = 0; i < Size; i++) {
      Vals[i] += Other[i];
    }
  }

  VecT operator+(const VecT &Other) {
    VecT Res(*this);
    Res += Other;
    return Res;
  }

  void operator-=(const VecT &Other) {
    assert(Size == Other.Size);
    for (size_t i = 0; i < Size; i++) {
      Vals[i] -= Other[i];
    }
  }

  VecT operator-(const VecT &Other) {
    VecT Res(*this);
    Res -= Other;
    return Res;
  }

  friend raw_ostream &operator<<(raw_ostream &OS, VecT V) {
    OS << "[ ";
    for (T Val : V.Vals) {
      OS << formatv("{0:f}", Val) << " ";
    }
    OS << "]";
    return OS;
  }

private:
  size_t Size;
  std::vector<T> Vals;
};

template <typename T, typename Iterator>
Vector<T> centroid(Iterator Begin, Iterator End) {
  Vector<T> C(*Begin);
  int Count = 1;
  for (auto It = Begin + 1; It != End; ++It) {
    C += *It;
    Count++;
  }
  C /= Count;
  return C;
}

template <typename T> struct VectorMapping {
  explicit VectorMapping(KnobSet *Knobs) { remap(Knobs); }

  void remap(KnobSet *Knobs) {
    this->Knobs = Knobs;
    KnobToIndex.clear();
    size_t Index = 0;
    for (auto &K : Knobs->IntKnobs) {
      KnobToIndex[K.first] = Index++;
    }
    for (auto &K : Knobs->LoopKnobs) {
      KnobToIndex[K.first] = Index;
      Index += LoopTransformConfig::NUM_PARAMS;
    }
  }

  Vector<T> map(KnobConfig &Cfg) {
    Vector<T> Vec(Cfg.getNumDimensions());
    for (auto &Entry : Cfg.IntCfg) {
      Vec[KnobToIndex[Entry.first]] = static_cast<T>(Entry.second);
    }
    for (auto &Entry : Cfg.LoopCfg) {
      auto i0 = KnobToIndex[Entry.first];
      for (auto i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
        Vec[i0 + i] = Entry.second.Vals[i];
      }
    }
    return Vec;
  }

  KnobConfig unmap(Vector<T> &Vec) {
    KnobConfig Cfg;
    for (auto IK : Knobs->IntKnobs) {
      Cfg.IntCfg[IK.first] = static_cast<int>(Vec[KnobToIndex[IK.first]]);
    }
    for (auto LK : Knobs->LoopKnobs) {
      auto &LC = Cfg.LoopCfg[LK.first];
      auto i0 = KnobToIndex[LK.first];
      for (auto i = 0; i < LoopTransformConfig::NUM_PARAMS; i++) {
        LC.Vals[i] = static_cast<unsigned>(Vec[i0 + i]);
      }
    }
    return Cfg;
  }

private:
  SmallDenseMap<KnobID, size_t> KnobToIndex;
  KnobSet *Knobs;
};

} // namespace tuner

#endif // CLANG_CONFIGMATH_H
