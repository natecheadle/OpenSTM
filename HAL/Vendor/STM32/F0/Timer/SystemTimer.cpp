#include "SystemTimer.h"

#include <stm32f0xx_ll_cortex.h>
#include <stm32f0xx_ll_exti.h>
#include <stm32f0xx_ll_rcc.h>
#include <stm32f0xx_ll_system.h>
#include <stm32f0xx_ll_utils.h>

#include <cassert>
#include <chrono>

namespace {
std::uint32_t sTickCount = 0;

openstm::hal::stmicro::f0::SystemTimer* pInstance = nullptr;

}  // namespace

namespace openstm::hal::stmicro::f0 {

SystemTimer::SystemTimer() { assert(pInstance == nullptr); }

SystemTimer::~SystemTimer() {
  if (pInstance == this) {
    pInstance = nullptr;
  }
}

SystemTimer::SystemTimer(SystemTimer&&) noexcept { pInstance = this; }

void SystemTimer::Initialize(std::chrono::microseconds tickPeriod) {
  m_PerTickTime = tickPeriod;
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_1);
  while (LL_FLASH_GetLatency() != LL_FLASH_LATENCY_1) {
  }
  LL_RCC_HSI_Enable();

  /* Wait till HSI is ready */
  while (LL_RCC_HSI_IsReady() != 1) {
  }
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI48_Enable();

  /* Wait till HSI48 is ready */
  while (LL_RCC_HSI48_IsReady() != 1) {
  }
  LL_RCC_PLL_ConfigDomain_SYS(LL_RCC_PLLSOURCE_HSI, LL_RCC_PLL_MUL_6,
                              LL_RCC_PREDIV_DIV_1);
  LL_RCC_PLL_Enable();

  /* Wait till PLL is ready */
  while (LL_RCC_PLL_IsReady() != 1) {
  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_PLL);

  /* Wait till System clock is ready */
  while (LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_PLL) {
  }
  LL_SetSystemCoreClock(48000000);

  LL_InitTick(ProcessorFrequncy,
              static_cast<std::uint32_t>(tickPeriod.count()));
  LL_SetSystemCoreClock(ProcessorFrequncy);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART2_CLKSOURCE_PCLK1);
  NVIC_EnableIRQ(SysTick_IRQn);
  LL_SYSTICK_EnableIT();
  LL_RCC_SetUSBClockSource(LL_RCC_USB_CLKSOURCE_HSI48);
}

std::uint32_t SystemTimer::TickCount() const { return sTickCount; }

std::chrono::microseconds SystemTimer::MicroSecondsPerTick() const {
  return m_PerTickTime;
}

ISystemTimer::TickOccurredSub SystemTimer::AttachToInterrupt(
    std::function<void(std::uint32_t)> f) {
  return m_TickOccurredEvent.Subscribe(std::move(f));
}

void SystemTimer::TickOccurred(std::uint32_t count) {
  m_TickOccurredEvent.Invoke(count);
}
}  // namespace openstm::hal::stmicro::f0

extern "C" void SysTick_Handler(void) {
  __disable_irq();
  std::uint32_t ticks = ++sTickCount;
  __enable_irq();
  pInstance->TickOccurred(ticks);
}
