//  Copyright (c) 2022, PickNik Inc.
//  All rights reserved.
//
//  Redistribution and use in source and binary forms, with or without
//  modification, are permitted provided that the following conditions
//  are met:
//
//   * Redistributions of source code must retain the above copyright
//     notice, this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above
//     copyright notice, this list of conditions and the following
//     disclaimer in the documentation and/or other materials provided
//     with the distribution.
//   * Neither the name of the copyright holder nor the names of its
//     contributors may be used to endorse or promote products derived
//     from this software without specific prior written permission.
//
//  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
//  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
//  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
//  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
//  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
//  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
//  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
//  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
//  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
//  POSSIBILITY OF SUCH DAMAGE.

/* Author: Sebastian Jahr
   Description: A random number generator singleton implementation
*/

#pragma once

#include <random>      // for std::mt19937, random_device , seed_seq
#include <array>       // for std::array, std::data,
#include <algorithm>   // for std::generate_n
#include <functional>  // for std::ref

namespace shapes
{
// Singleton random number generator class based on https://www.modernescpp.com/index.php/thread-safe-initialization-of-a-singleton
class RandomNumberGenerator
{
private:
  std::mt19937 generator_;

  // Delete copy constructor
  RandomNumberGenerator(const RandomNumberGenerator&) = delete;
  // Don't allow assigning the instance of this class
  RandomNumberGenerator& operator=(const RandomNumberGenerator&) = delete;

  RandomNumberGenerator()
    : generator_{ []() {
      std::array<int, std::mt19937::state_size> seed_data;
      std::random_device random_device;
      std::generate_n(std::data(seed_data), std::size(seed_data), std::ref(random_device));
      std::seed_seq sequence(std::begin(seed_data), std::end(seed_data));
      std::mt19937 generator(sequence);
      return generator;
    }() } {};

  RandomNumberGenerator(std::seed_seq& seed_sequence)
    : generator_{ [](std::seed_seq& seed_sequence) {
      std::mt19937 generator(seed_sequence);
      return generator;
    }(seed_sequence) } {};
  ~RandomNumberGenerator() = default;

public:
  static RandomNumberGenerator& getInstance()
  {
    // Static valiables with blocked scopes will be only created once
    thread_local RandomNumberGenerator instance;
    return instance;
  }

  static RandomNumberGenerator& getInstance(std::seed_seq& seed_sequence)
  {
    // Static valiables with blocked scopes will be only created once
    thread_local RandomNumberGenerator instance(seed_sequence);
    return instance;
  }

  /// \brief Return a randum number based on a bounded uniform distribution
  auto uniform(double const lower_bound, double const upper_bound)
  {
    std::uniform_real_distribution<double> distribution(lower_bound, upper_bound);
    return distribution(generator_);
  }
};
}  // namespace shapes
