#pragma once
#include <App.h>
#include <Vendor/STMicro/F3/IO/DigitalIn.h>
#include <Vendor/STMicro/F3/IO/DigitalOut.h>
#include <Vendor/STMicro/F3/Serial/USART.h>
#include <Vendor/STMicro/F3/Timer/SystemTimer.h>

#include <IO/Digital/DigitalIn.hpp>
#include <IO/Digital/DigitalOut.hpp>
#include <Serial/USART.hpp>
#include <Timer/SystemTimer.hpp>
#include <functional>

namespace openstm::hal::boards::nucleo64f303re {

class App : public hal::App {
  std::function<void()> m_UpdateFunc;

 public:
  static DigitalOut<stmicro::f3::DigitalOut> LED2;
  static DigitalIn<stmicro::f3::DigitalIn> Button1;
  static SystemTimer<stmicro::f3::SystemTimer> SysTimer;
  static USART<stmicro::f3::USART> USB_USART;

  App(std::function<void()> update);

 protected:
  void Initialize() override;
  void Update() override;
};
}  // namespace openstm::hal::boards::nucleo64f303re
