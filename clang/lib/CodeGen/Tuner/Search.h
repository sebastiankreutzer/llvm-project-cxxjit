//
// Created by sebastian on 27.01.20.
//

#ifndef LLVM_SEARCH_H
#define LLVM_SEARCH_H

#include <llvm/Support/Error.h>
#include "Math.h"

#include <type_traits>
#include <random>

namespace clang {
namespace jit {

enum class ParamType {
  INT, FP
};

class WrongParamType : public llvm::ErrorInfo<WrongParamType> {
public:
  static char ID;
  ParamType Type;

  explicit WrongParamType(ParamType Type) : Type(Type) {
  }

  void log(llvm::raw_ostream &OS) const override {
    OS << "Wrong parameter type: " << (Type == ParamType::INT ? "INT" : "FP");
  }

  std::error_code convertToErrorCode() const override {
    return std::make_error_code(std::errc::invalid_argument);
  }

};

struct ParamVal final {

  using IntType = long;
  using FPType = double;

  ParamVal(ParamType Type) : Type(Type) {
    clear();
  }

  ParamVal() : ParamVal(ParamType::INT) {

  }

  template<typename T, std::enable_if_t<std::is_floating_point<T>::value, int> = 0>
  ParamVal(T Val) : Type(ParamType::FP) {
    this->Val.FP_Val = static_cast<FPType>(Val);
  }

  template<typename T, std::enable_if_t<std::is_integral<T>::value, int> = 0>
  ParamVal(T Val) : Type(ParamType::INT) {
    this->Val.Int_Val = static_cast<IntType>(Val);
  }

  ParamVal(const ParamVal& Other) = default;

  ParamVal& operator=(const ParamVal& Other) {
    assert(Other.Type == this->Type && "Incompatible types");
    Val = Other.Val;
    return *this;
  }

  bool operator==(const ParamVal& Other) const {
    return Other.Type == Type &&
               Type == ParamType::INT ? Val.Int_Val == Other.Val.Int_Val : Val.FP_Val == Other.Val.FP_Val;
  }

  bool operator<(const ParamVal& Other) const {
    assert(Type == Other.Type && "Can't compare values of different type");
    return Type == ParamType::INT ? Val.Int_Val < Other.Val.Int_Val : Val.FP_Val < Other.Val.FP_Val;
  }

  bool operator<=(const ParamVal& Other) const {
    assert(Type == Other.Type && "Can't compare values of different type");
    return Type == ParamType::INT ? Val.Int_Val <= Other.Val.Int_Val : Val.FP_Val <= Other.Val.FP_Val;
  }

  bool operator>(const ParamVal& Other) const {
    return !operator<=(Other);
  }

  bool operator>=(const ParamVal& Other) const {
    return !operator<(Other);
  }

  ParamVal operator+(const ParamVal& Other) const {
    ParamVal Sum(*this);
    Sum += Other;
    return Sum;
  }

  void operator+=(const ParamVal& Other) {
    assert(Other.Type == this->Type && "Incompatible types");
    switch(Type) {
      case ParamType::INT:
        Val.Int_Val += Other.Val.Int_Val;
        break;
      case ParamType::FP:
        Val.FP_Val += Other.Val.FP_Val;
      default:
        llvm_unreachable("");
    }
  }

  ParamVal operator-(const ParamVal& Other) const {
    ParamVal Sum(*this);
    Sum -= Other;
    return Sum;
  }

  void operator-=(const ParamVal& Other) {
    assert(Other.Type == this->Type && "Incompatible types");
    switch(Type) {
      case ParamType::INT:
        Val.Int_Val -= Other.Val.Int_Val;
        break;
      case ParamType::FP:
        Val.FP_Val -= Other.Val.FP_Val;
      default:
        llvm_unreachable("");
    }
  }


  void clear() {
    switch(Type) {
      case ParamType::INT:
        Val.Int_Val = 0;
        break;
      case ParamType ::FP:
        Val.FP_Val = 0;
        break;
      default:
        llvm_unreachable("");
    }
  }

  llvm::Expected<IntType> getIntVal() const {
    if (Type == ParamType::INT)
      return Val.Int_Val;
    return llvm::make_error<WrongParamType>(Type);
  }

  llvm::Expected<FPType> getFPVal() const {
    if (Type == ParamType::FP)
      return Val.FP_Val;
    return llvm::make_error<WrongParamType>(Type);
  }

  double getAsDouble() const {
    switch(Type) {
      case ParamType::FP:
        return Val.FP_Val;
      case ParamType::INT:
        return Val.Int_Val;
    }
    llvm_unreachable("");
  }

  friend llvm::raw_ostream& operator<<(llvm::raw_ostream& OS, const ParamVal& Val);

  const ParamType Type;
private:
  union ValType {
    FPType FP_Val;
    IntType Int_Val;
  };
  ValType Val;

};


struct SearchDim {

  using ValT = ParamVal;

  SearchDim(ValT Min, ValT Max, ValT Default, llvm::StringRef Name = "") : Min(Min), Max(Max), Default(Default), Name(Name) {
    assert(Min.Type == Max.Type && Min.Type == Default.Type && "Inconsistent parameter types");
    Type = Min.Type;
  }

//  template<typename T>
//  SearchDim(ParamType Type, T Min, T Max, T Default, llvm::StringRef Name = "") : Type(Type), Min(Type), Max(Type), Default(Type), Name(Name) {
//    this->Min = Min;
//    this->Max = Max;
//    this->Default = Default;
//  }

  ValT Min;
  ValT Max;
  ValT Default;
  ParamType Type;
  llvm::SmallString<16> Name;
};

//template<typename TargetT>
//class MappedParameter: SearchDim {
//
//};

class SearchSpace {
  using VecT = llvm::SmallVector<SearchDim, 4>;
public:
  SearchSpace(llvm::SmallVector<SearchDim, 4> Params = {}) : Params(std::move(Params)) {}

  void addDim(SearchDim Dim) {
    Params.push_back(std::move(Dim));
  }

  VecT::iterator begin() {
    return Params.begin();
  }

  VecT::const_iterator begin() const {
    return Params.begin();
  }

  VecT::iterator end() {
    return Params.end();
  }

  VecT::const_iterator end() const {
    return Params.end();
  }

  SearchDim& getDim(unsigned Idx) {
    assert(Idx < Params.size() && "Invalid search dimensions");
    return Params[Idx];
  }


  const SearchDim& getDim(unsigned Idx) const {
    assert(Idx < Params.size() && "Invalid search dimensions");
    return Params[Idx];
  }


  SearchDim& operator[](unsigned Idx) {
    return getDim(Idx);
  }

  const SearchDim& operator[](unsigned Idx) const {
    return getDim(Idx);
  }

  unsigned getNumDimensions() const {
    return Params.size();
  }

  bool empty() const {
    return Params.empty();
  }
private:
  VecT Params;
};

template<typename T>
bool isLegal(const SearchSpace& Space, const Vector<T> &Vec) {
  for (auto I = 0; I < Space.getNumDimensions(); I++) {
    if (Vec[I] < Space.getDim(I).Min || Vec[I] > Space.getDim(I).Max) {
      return false;
    }
  }
  return true;
}

template<typename T>
void legalize(const SearchSpace& Space, Vector<T> &Vec) {
  for (auto I = 0; I < Space.getNumDimensions(); I++) {
    Vec[I] = std::min(std::max(Vec[I], static_cast<T>(Space[I].Min.getAsDouble()),
                               static_cast<T>(Space[I].Max.getAsDouble())));
  }
}

struct ParamConfig {
  using ValT = ParamVal;
  using VecT = Vector<ValT>;

  ParamConfig() : Space(nullptr), Values(0) {
  }

  explicit ParamConfig(const SearchSpace& Space) : Space(&Space), Values(Space.getNumDimensions())  {
  }

//  ParamConfig(VecT Vals, const SearchSpace* Space) : Space(Space), Values(std::move(Vals)) {
//  }


  ValT& operator[] (size_t Index) {
    return Values[Index];
  }

  const ValT& operator[] (size_t Index) const {
    return Values[Index];
  }

  typename VecT::iterator begin() {
    return Values.begin();
  }

  typename VecT::const_iterator begin() const {
    return Values.begin();
  }

  typename VecT::iterator end() {
    return Values.end();
  }

  typename VecT::const_iterator end() const {
    return Values.end();
  }

  bool isLegal() const {
    if (!Space)
      return true;
    return clang::jit::isLegal<ParamVal>(*Space, Values);
  }

  bool isEmpty() const {
    return Values.size() == 0;
  }

  size_t size() const {
    return Values.size();
  }

  void dump(llvm::raw_ostream &OS) const;

  void dump() const {
    dump(llvm::outs());
  }

  const SearchSpace* Space;
  Vector<ValT> Values;
};

inline ParamConfig createDefaultConfig(const SearchSpace& Space) {
  ParamConfig Cfg(Space);
  for (auto I = 0; I < Space.getNumDimensions(); I++) {
    Cfg[I] = Space.getDim(I).Default;
  }
  return Cfg;
}

template<typename RNETy>
ParamConfig createRandomConfig(RNETy &RNE, const SearchSpace& Space) {
  ParamConfig Cfg(Space);
  auto I = 0;
  for (auto& Dim : Space) {
    switch(Dim.Type) {
      case ParamType::INT: {
        std::uniform_int_distribution<ParamVal::IntType> IntDist(cantFail(Dim.Min.getIntVal()), cantFail(Dim.Max.getIntVal()));
        Cfg[I] = IntDist(RNE);
        break;
      }
      case ParamType::FP: {
        std::uniform_real_distribution<ParamVal::FPType> FloatDist(cantFail(Dim.Min.getFPVal()), cantFail(Dim.Max.getFPVal()));
        Cfg[I] = FloatDist(RNE);
        break;
      }
      default:
        llvm_unreachable("");
    }
    I++;
  }
  return Cfg;
}


template<typename T>
inline ParamConfig createConfig(SearchSpace& Space, const Vector<T> &Vec) {
  ParamConfig Cfg(Space);
  for (auto I = 0; I < Space.getNumDimensions(); I++) {
    auto& Param = Space[I];
    switch(Param.Type) {
      case ParamType::INT:
        Cfg[I] = static_cast<ParamVal::IntType>(Vec[I]);
        break;
      case ParamType::FP:
        Cfg[I] = static_cast<ParamVal::FPType>(Vec[I]);
        break;
      default:
        llvm_unreachable("");
    }
  }
  return Cfg;
}

template<typename T>
Vector<T> convertTo(const ParamConfig& Cfg) {
  auto Size = Cfg.Space->getNumDimensions();
  Vector<T> Vec(Size);
  for (auto I = 0; I < Size; I++) {
    Vec[I] = static_cast<T>(Cfg[I].getAsDouble());
  }
  return Vec;
}

}
}

#endif //LLVM_SEARCH_H
