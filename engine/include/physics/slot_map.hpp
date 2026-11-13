#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

namespace physics {

/// Generational slot storage. Objects live behind unique_ptr so raw pointers
/// stay valid when other objects are added or removed.
template <typename T, typename Id>
class SlotMap {
 public:
  /// Returns an invalid id when the index space (16 bits) is exhausted.
  Id insert(std::unique_ptr<T> object) {
    Id id;
    if (!free_.empty()) {
      id.index = free_.back();
      free_.pop_back();
    } else if (slots_.size() <= Id::kMaxIndex) {
      id.index = static_cast<std::uint32_t>(slots_.size());
      slots_.emplace_back();
    } else {
      return Id();
    }
    Slot& slot = slots_[id.index];
    slot.object = std::move(object);
    id.generation = slot.generation;
    ++count_;
    return id;
  }

  bool remove(Id id) {
    if (get(id) == nullptr) {
      return false;
    }
    Slot& slot = slots_[id.index];
    slot.object.reset();
    slot.generation += 1;
    if (slot.generation > Id::kMaxGeneration) {
      slot.generation = 1;
    }
    free_.push_back(id.index);
    --count_;
    return true;
  }

  T* get(Id id) const {
    if (id.generation == 0 || id.index >= slots_.size()) {
      return nullptr;
    }
    const Slot& slot = slots_[id.index];
    if (slot.generation != id.generation) {
      return nullptr;
    }
    return slot.object.get();
  }

  std::size_t size() const { return count_; }

  /// Visits live objects in slot order: f(Id, T&).
  template <typename F>
  void forEach(F&& f) const {
    for (std::uint32_t i = 0; i < slots_.size(); ++i) {
      const Slot& slot = slots_[i];
      if (slot.object) {
        Id id;
        id.index = i;
        id.generation = slot.generation;
        f(id, *slot.object);
      }
    }
  }

 private:
  struct Slot {
    std::unique_ptr<T> object;
    std::uint32_t generation = 1;
  };
  std::vector<Slot> slots_;
  std::vector<std::uint32_t> free_;
  std::size_t count_ = 0;
};

}  // namespace physics
