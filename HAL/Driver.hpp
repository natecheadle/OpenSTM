#pragma once

namespace openstm::hal {

template <typename T>
class Driver {
  T m_Device;

 public:
  Driver(T&& device) : m_Device(std::forward<T>(device)) {}

  T& Device() { return m_Device; }
  const T& Device() const { return m_Device; }
};
}  // namespace openstm::hal
