//
// Created by sebastian on 27.01.20.
//

#ifndef LLVM_MATH_H
#define LLVM_MATH_H

namespace clang {
namespace jit {

// TODO: Inefficient vector implementation, but will do for now
template <typename T> class Vector {
  using VecT = Vector<T>;

public:
  using Container = llvm::SmallVector<T, 4>;

  using iterator = typename Container::iterator;
  using const_iterator = typename Container::const_iterator;

  explicit Vector(size_t N) : Size(N), Vals(N) {}

  Vector() : Size(1), Vals(1){};

  Vector(const Vector &Other) = default;

  Vector<T> &operator=(const Vector &Other) = default;

  size_t size() const { return Size; }

  T &operator[](size_t I) { return Vals[I]; }

  const T &operator[](size_t I) const { return Vals[I]; }

  iterator begin() { return Vals.begin(); }

  const_iterator begin() const { return Vals.begin(); }

  iterator end() { return Vals.end(); }

  const_iterator end() const { return Vals.end(); }

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

  bool equals(const VecT &Other, double eps) {
    if (Size != Other.Size)
      return false;
    for (size_t i = 0; i < Size; i++) {
      if (std::abs(Vals[i] - Other[i]) > eps)
        return false;
    }
    return true;
  }

  friend llvm::raw_ostream &operator<<(llvm::raw_ostream &OS, VecT V) {
    OS << "[ ";
    for (T Val : V.Vals) {
      OS << formatv("{0:f}", Val) << " ";
    }
    OS << "]";
    return OS;
  }

private:
  size_t Size;
  Container Vals;
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

}
}

#endif //LLVM_MATH_H
