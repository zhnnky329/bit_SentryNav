// Copyright 2026 bw_nav contributors
// SPDX-License-Identifier: Apache-2.0
//
// Content digest helper used to decide whether a cached pose still belongs to
// the currently loaded maps.

#ifndef RELOCALIZER__FILE_DIGEST_HPP_
#define RELOCALIZER__FILE_DIGEST_HPP_

#include <cstdint>
#include <fstream>
#include <string>

namespace relocalizer
{

/// FNV-1a digest of a file's bytes.  Returns 0 when the file cannot be read.
inline uint64_t digest_of_file(const std::string & path)
{
  std::ifstream stream(path, std::ios::binary);
  if (!stream) {
    return 0;
  }
  uint64_t hash = 1469598103934665603ULL;
  char buffer[65536];
  while (stream.good()) {
    stream.read(buffer, sizeof(buffer));
    const std::streamsize got = stream.gcount();
    for (std::streamsize index = 0; index < got; ++index) {
      hash ^= static_cast<unsigned char>(buffer[index]);
      hash *= 1099511628211ULL;
    }
  }
  return hash;
}

}  // namespace relocalizer

#endif  // RELOCALIZER__FILE_DIGEST_HPP_
