#pragma once

#include <array>
#include <cstddef>
#include <limits>

namespace openstm::lib {
template <typename T, size_t SIZE>
class RingBuffer {
  static_assert(SIZE > 1, "SIZE must be greater than 1");
  static_assert(SIZE < std::numeric_limits<size_t>::max(),
                "SIZE must be less than size_t max");
  std::array<T, SIZE> m_Data;
  size_t m_Begin{0};
  size_t m_End{0};

 public:
  bool IsEmpty() const { return m_Begin == m_End; }
  bool IsFull() const {
    if (m_Begin == m_End) {
      return false;
    }
    if (m_End > m_Begin) {
      return m_Begin == 0 && m_End == m_Data.size() - 1;
    }
    return m_Begin - 1 == m_End;
  }

  void Clear() { m_Begin = m_End; }

  size_t MaxCount() const { return SIZE - 1; }

  size_t RemainingSlots() const { return MaxCount() - BufferedCount(); }

  size_t BufferedCount() const {
    if (m_End < m_Begin) {
      return m_End + SIZE - m_Begin;
    } else {
      return m_End - m_Begin;
    }
  }

  bool Next(T& out) const {
    if (IsEmpty()) {
      return false;
    }
    out = m_Data[m_Begin];
    return true;
  }

  bool Last(T& out) const {
    if (IsEmpty()) {
      return false;
    }
    size_t last = m_End;
    if (last == 0) {
      last = m_Data.size() - 1;
    } else {
      last--;
    }
    out = m_Data[last];
    return true;
  }

  bool Push(const T& val) {
    if (!IsFull()) {
      m_Data[m_End] = val;
      m_End++;
      if (m_End == m_Data.size()) {
        m_End = 0;
      }
      return true;
    }
    return false;
  }

  bool Pop(T& out) {
    if (!IsEmpty()) {
      out = m_Data[m_Begin];
      m_Begin++;
      if (m_Begin == m_Data.size()) {
        m_Begin = 0;
      }
      return true;
    }
    return false;
  }
};
}  // namespace openstm::lib
