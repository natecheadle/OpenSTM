#pragma once
#include <App.h>
#include <Vendor/STM32/F0/IO/DigitalIn.h>
#include <Vendor/STM32/F0/IO/DigitalOut.h>
#include <Vendor/STM32/F0/Serial/USART.h>
#include <Vendor/STM32/F0/Timer/SystemTimer.h>

#include <IO/Digital/DigitalIn.hpp>
#include <IO/Digital/DigitalOut.hpp>
#include <Serial/USART.hpp>
#include <Timer/SystemTimer.hpp>
#include <functional>

namespace openstm::hal::boards::nucleof072discovery {

class App : public hal::App {
  std::function<void()> m_UpdateFunc;

 public:
  static DigitalOut<stmicro::f0::DigitalOut> LED2;
  static DigitalOut<stmicro::f0::DigitalOut> RedLED;
  static DigitalOut<stmicro::f0::DigitalOut> OrangeLED;
  static DigitalOut<stmicro::f0::DigitalOut> BlueLED;
  static DigitalIn<stmicro::f0::DigitalIn> Button1;
  static SystemTimer<stmicro::f0::SystemTimer> SysTimer;
  static USART<stmicro::f0::USART> USB_USART;

  App(std::function<void()> update);

 protected:
  void Initialize() override;
  void Update() override;
};
}  // namespace openstm::hal::boards::nucleof072discovery
