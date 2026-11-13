/**
 * @brief Random helpers giving the same values on every platform.
 * The standard distributions are implementation-defined (a seed does not give the same numbers
 * with GCC, Clang and MSVC), std::mt19937 itself is not: these helpers only use its raw output.
 */

#pragma once
// Libraries
#include <cmath>  // To use log
#include <random> // To use std::mt19937

namespace rnd {
   /** @brief Uniform real number.
    * @returns Value in [0, 1) */
   inline double uniform(std::mt19937& rng) {
      return rng() / 4294967296.0; // 2^32
   }
   /** @brief Uniform integer.
    * @param n Number of values, must be positive
    * @returns Value in [0, n-1] */
   inline int index(std::mt19937& rng, const int n) {
      return static_cast<int>(uniform(rng) * n);
   }
   /** @brief Exponential law (time between 2 events of a Poisson process).
    * @param rate Events per unit of time, must be positive
    * @returns Time to the next event */
   inline double exponential(std::mt19937& rng, const double rate) {
      return -std::log(1.0 - uniform(rng)) / rate;
   }
}
