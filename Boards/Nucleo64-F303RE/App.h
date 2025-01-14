#pragma once
#include <App.h>
#include <Vendor/STMicro/F3/IO/DigitalIn.h>
#include <Vendor/STMicro/F3/IO/DigitalOut.h>

#include <IO/Digital/DigitalIn.hpp>
#include <IO/Digital/DigitalOut.hpp>
#include <functional>

namespace openstm::hal::boards::nucleo64f303re {

class App : public hal::App {
  std::function<void()> m_UpdateFunc;

 public:
  static DigitalOut<stmicro::f3::DigitalOut> LED2;
  static DigitalIn<stmicro::f3::DigitalIn> Button1;

  App(std::function<void()> update);

 protected:
  void Initialize() override;
  void Update() override;

 private:
  void InitGPIO();
  void InitUART();
  void ConfigSysClock();
};
}  // namespace openstm::hal::boards::nucleo64f303re
