#include "App.h"

#include <stm32f0xx_ll_bus.h>
#include <stm32f0xx_ll_exti.h>
#include <stm32f0xx_ll_gpio.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_system.h>
#include <stm32f0xx_ll_usart.h>
#include <stm32f0xx_ll_utils.h>

#include "IO/Digital/DigitalOut.hpp"
#include "Vendor/STM32/F0/IO/DigitalOut.h"

namespace {
#define NVIC_PRIORITYGROUP_0                                   \
  ((uint32_t)0x00000007) /*!< 0 bit  for pre-emption priority, \
                              4 bits for subpriority */
#define NVIC_PRIORITYGROUP_1                                   \
  ((uint32_t)0x00000006) /*!< 1 bit  for pre-emption priority, \
                              3 bits for subpriority */
#define NVIC_PRIORITYGROUP_2                                   \
  ((uint32_t)0x00000005) /*!< 2 bits for pre-emption priority, \
                              2 bits for subpriority */
#define NVIC_PRIORITYGROUP_3                                   \
  ((uint32_t)0x00000004) /*!< 3 bits for pre-emption priority, \
                              1 bit  for subpriority */
#define NVIC_PRIORITYGROUP_4                                   \
  ((uint32_t)0x00000003) /*!< 4 bits for pre-emption priority, \
                              0 bit  for subpriority */
}  // namespace

namespace openstm::hal::boards::nucleof072discovery {
DigitalOut<stmicro::f0::DigitalOut> App::LED2 =
    DigitalOut(stmicro::f0::DigitalOut(PinID::Nine, GPIOC));
DigitalOut<stmicro::f0::DigitalOut> App::RedLED =
    DigitalOut(stmicro::f0::DigitalOut(PinID::Six, GPIOC));
DigitalOut<stmicro::f0::DigitalOut> App::OrangeLED =
    DigitalOut(stmicro::f0::DigitalOut(PinID::Eight, GPIOC));
DigitalOut<stmicro::f0::DigitalOut> App::BlueLED =
    DigitalOut(stmicro::f0::DigitalOut(PinID::Seven, GPIOC));

DigitalIn<stmicro::f0::DigitalIn> App::Button1 =
    DigitalIn(stmicro::f0::DigitalIn(PinID::Zero, GPIOA));

SystemTimer<stmicro::f0::SystemTimer> App::SysTimer =
    SystemTimer(stmicro::f0::SystemTimer());

USART<stmicro::f0::USART> App::USB_USART = USART(stmicro::f0::USART(
    PinID::Ten, PinID::Nine, GPIOA, USART2, 38400, 4, 32, true, true));

App::App(std::function<void()> update) : m_UpdateFunc(update) {}

void App::Initialize() {
  LL_APB1_GRP2_EnableClock(LL_APB1_GRP2_PERIPH_SYSCFG);
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_PWR);

  NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);

  SysTimer.Initialize(std::chrono::milliseconds(1));
  BlueLED.Initialize();
  LED2.Initialize();
  RedLED.Initialize();
  OrangeLED.Initialize();
  Button1.Initialize();
  USB_USART.Initialize();
}

void App::Update() { m_UpdateFunc(); }
}  // namespace openstm::hal::boards::nucleof072discovery
