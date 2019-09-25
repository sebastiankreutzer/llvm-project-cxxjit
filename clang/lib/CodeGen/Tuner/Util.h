//
// Created by sebastian on 25.09.19.
//

#ifndef CLANG_UTIL_H
#define CLANG_UTIL_H

#include <chrono>

namespace tuner {

namespace util {

inline unsigned genSeed() {
  return static_cast<unsigned>(std::chrono::system_clock::now().time_since_epoch().count());
}

}

}

#endif //CLANG_UTIL_H
