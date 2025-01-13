#pragma once

#include <type_traits>

#include "App.h"

namespace openstm::hal {

template <typename T>
class AppWrapper {
  static_assert(std::is_base_of_v<App, T>, "T must implement IDigitalOut");
  T m_ConcreteApp;

 public:
  AppWrapper(const T& concreteApp) : m_ConcreteApp(concreteApp) {}
  void Run() { m_ConcreteApp.Run(); }
};
}  // namespace openstm::hal