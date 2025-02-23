#pragma once

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>

namespace openstm::lib {

template <size_t MAX_SIZE>
class StaticStackAllocator {
  static std::array<std::uint8_t, MAX_SIZE> m_Data;
  static std::atomic<size_t> m_NextLocation;

 public:
  StaticStackAllocator() = default;

  StaticStackAllocator(const StaticStackAllocator& other) = delete;
  StaticStackAllocator(StaticStackAllocator&& other) noexcept = delete;

  StaticStackAllocator& operator=(const StaticStackAllocator& other) = delete;
  StaticStackAllocator& operator=(StaticStackAllocator&& other) noexcept =
      delete;

  static void* Mallocate(size_t size) {
    void* pObjNew{nullptr};
    size_t currentLoc = m_NextLocation;
    do {
      if (currentLoc >= m_Data.size()) {
        return nullptr;
      }
      if (currentLoc + size > m_Data.size()) {
        return nullptr;
      }
      pObjNew = m_Data.data() + currentLoc;
    } while (
        !m_NextLocation.compare_exchange_weak(currentLoc, currentLoc + size));

    return pObjNew;
  }

  static void Free(void*) {
    // Do Nothing - This allocator does not support free
  }

  static size_t RemainingSpace() { return m_Data.size() - m_NextLocation; }

  static void Reset() { m_NextLocation = 0; }

  template <typename T>
  static T* allocate(size_t n) {
    return reinterpret_cast<T*>(Mallocate(n * sizeof(T)));
  }

  template <typename T>
  static T* allocate(size_t n, const void*) {
    return allocate<T>(n);
  }

  template <typename T>
  static void deallocate(T* obj, size_t) {
    return Free(obj);
  }
};

template <typename T, size_t MAX_SIZE>
class StaticStackAllocatorT {
 public:
  using value_type = T;

  static T* allocate(size_t n) {
    return StaticStackAllocator<MAX_SIZE>::template allocate<T>(n);
  }

  static T* allocate(size_t n, const void* hint) {
    return StaticStackAllocator<MAX_SIZE>::template allocate<T>(n, hint);
  }

  static void deallocate(T* obj, size_t size) {
    return StaticStackAllocator<MAX_SIZE>::template deallocate<T>(obj, size);
  }

  template <typename U>
  struct rebind {
    typedef StaticStackAllocatorT<U, MAX_SIZE> other;
  };
};

template <size_t MAX_SIZE>
std::array<std::uint8_t, MAX_SIZE> StaticStackAllocator<MAX_SIZE>::m_Data = {};

template <size_t MAX_SIZE>
std::atomic<size_t> StaticStackAllocator<MAX_SIZE>::m_NextLocation = 0;
}  // namespace openstm::lib
