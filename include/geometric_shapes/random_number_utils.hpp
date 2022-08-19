// Copyright (c) 2022, PickNik Inc.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//    * Redistributions of source code must retain the above copyright
//      notice, this list of conditions and the following disclaimer.
//
//    * Redistributions in binary form must reproduce the above copyright
//      notice, this list of conditions and the following disclaimer in the
//      documentation and/or other materials provided with the distribution.
//
//    * Neither the name of the Willow Garage, Inc. nor the names of its
//      contributors may be used to endorse or promote products derived from
//      this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.

/* Author: Sebastian Jahr
   Description: A random number generator singleton implementation
*/

#pragma once

#include <random>       // for std::mt19937, random_device , seed_seq
#include <array>        // for std::array, std::data,
#include <algorithm>    // for std::generate_n
#include <functional>   // for std::ref
#include "Eigen/Dense"  // for Eigen::Quaterniond::UnitRandom();
#include <time.h>       // for time
#include <optional>     // for optional

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
    }() }
  {
    // Seed srand initially
    std::srand(time(NULL));
  };

  RandomNumberGenerator(std::seed_seq& seed_sequence)
    : generator_{ [](std::seed_seq& seed_sequence) {
      std::mt19937 generator(seed_sequence);
      return generator;
    }(seed_sequence) } {};
  ~RandomNumberGenerator() = default;

public:
  /** The first time this function is called it creates a thread_local random number generator.
   * If a seed sequence is provided on that first call it is used to create the generator, otherwise the random device
   * is used to seed the generator. After the first call to this function, if a seed sequence is provided this function
   * throws.
   */
  [[nodiscard]] static RandomNumberGenerator& getInstance(std::optional<std::seed_seq> seed_sequence = std::nullopt)
  {
    bool first = false;
    thread_local RandomNumberGenerator instance = [&seed_sequence, &first]() {
      first = true;
      if (seed_sequence.has_value())
      {
        return RandomNumberGenerator(seed_sequence.value());
      }
      else
      {
        return RandomNumberGenerator();
      }
    }();
    if (!first && seed_sequence.has_value())
    {
      throw std::runtime_error("RandomNumberGenerator cannot be re-seeded");
    }
    return instance;
  }

  /// \brief Return a random number based on a bounded uniform distribution
  [[nodiscard]] auto uniform(double const lower_bound, double const upper_bound)
  {
    std::uniform_real_distribution<double> distribution(lower_bound, upper_bound);
    return distribution(generator_);
  }

  /// \brief Return a random quaternion. The reason for this wrapper function is that we need to make sure, that srand()
  /// is seeded before UnitRandom() is called. The seeding takes place in the Singleton's constructor.
  [[nodiscard]] auto getRandomQuaternion(std::optional<unsigned int> seed = std::nullopt)
  {
    if (seed.has_value())
    {
      // Optional re-seed
      std::srand(seed.value());
    }
    return Eigen::Quaterniond::UnitRandom();
  }
};
}  // namespace shapes
