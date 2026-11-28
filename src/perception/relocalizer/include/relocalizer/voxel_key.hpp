// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Integer keys used by the spatial hash containers.  A single mixing function
// keeps the hash quality uniform across the 2-D and 3-D maps.

#ifndef RELOCALIZER__VOXEL_KEY_HPP_
#define RELOCALIZER__VOXEL_KEY_HPP_

#include <cstdint>
#include <cstddef>

namespace relocalizer
{

inline std::size_t mix_cell(int32_t i, int32_t j, int32_t k)
{
  uint64_t h = 1469598103934665603ULL;
  const uint32_t words[3] = {
    static_cast<uint32_t>(i), static_cast<uint32_t>(j), static_cast<uint32_t>(k)};
  for (uint32_t word : words) {
    h ^= static_cast<uint64_t>(word);
    h *= 1099511628211ULL;
    h ^= h >> 29;
  }
  return static_cast<std::size_t>(h);
}

struct CellKey3
{
  int32_t i{0};
  int32_t j{0};
  int32_t k{0};

  bool operator==(const CellKey3 & other) const
  {
    return i == other.i && j == other.j && k == other.k;
  }
};

struct CellKey3Hash
{
  std::size_t operator()(const CellKey3 & key) const
  {
    return mix_cell(key.i, key.j, key.k);
  }
};

struct CellKey2
{
  int32_t i{0};
  int32_t j{0};

  bool operator==(const CellKey2 & other) const
  {
    return i == other.i && j == other.j;
  }
};

struct CellKey2Hash
{
  std::size_t operator()(const CellKey2 & key) const
  {
    return mix_cell(key.i, key.j, 0);
  }
};

}  // namespace relocalizer

#endif  // RELOCALIZER__VOXEL_KEY_HPP_
