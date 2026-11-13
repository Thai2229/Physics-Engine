#pragma once

#include <cstdint>

namespace physics {

/// Stable handle: slot index plus a generation counter that is bumped whenever
/// the slot is freed, so stale handles are detected instead of aliasing a new
/// object. Generation 0 is never used, so a default handle is invalid.
/// The packed int form uses 15 generation bits and 16 index bits (always >= 0).
template <typename Tag>
struct Handle {
  std::uint32_t index = 0;
  std::uint32_t generation = 0;

  static constexpr std::uint32_t kMaxIndex = 0xFFFF;
  static constexpr std::uint32_t kMaxGeneration = 0x7FFF;

  bool isValid() const { return generation != 0; }

  /// Packed form for JS (-1 for an invalid handle).
  std::int32_t toInt() const {
    if (generation == 0) {
      return -1;
    }
    return static_cast<std::int32_t>((generation << 16) | index);
  }

  static Handle fromInt(std::int32_t v) {
    Handle h;
    if (v < 0) {
      return h;
    }
    h.index = static_cast<std::uint32_t>(v) & 0xFFFFu;
    h.generation = (static_cast<std::uint32_t>(v) >> 16) & 0x7FFFu;
    return h;
  }

  friend bool operator==(const Handle& a, const Handle& b) {
    return a.index == b.index && a.generation == b.generation;
  }
  friend bool operator!=(const Handle& a, const Handle& b) { return !(a == b); }
};

struct BodyTag {};
struct JointTag {};
using BodyId = Handle<BodyTag>;
using JointId = Handle<JointTag>;

}  // namespace physics
