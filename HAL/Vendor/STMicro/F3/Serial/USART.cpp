#include "USART.h"

#include <stm32f3xx_ll_bus.h>
#include <stm32f3xx_ll_gpio.h>
#include <stm32f3xx_ll_usart.h>

#include "../Timer/SystemTimer.h"

namespace openstm::hal::stmicro::f3 {
USART::USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox,
             USART_TypeDef* usart, std::uint32_t baudRate,
             const SystemTimer& timer)
    : m_GPIOx(gpiox),
      m_USART(usart),
      m_TXPin(txPin),
      m_RXPin(rxPin),
      m_BaudRate(baudRate),
      m_SysTimer(timer) {}

PinID USART::TXPin() const { return m_TXPin; }

PinID USART::RXPin() const { return m_RXPin; }

std::uint32_t USART::BaudRate() const { return m_BaudRate; }

void USART::Initialize() {
  LL_USART_InitTypeDef USART_InitStruct = {};

  LL_GPIO_InitTypeDef GPIO_InitStruct = {};

  /* Peripheral clock enable */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_USART2);

  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOA);
  /**USART2 GPIO Configuration
  PA2   ------> USART2_TX
  PA3   ------> USART2_RX
  */
  GPIO_InitStruct.Pin =
      static_cast<std::uint32_t>(m_TXPin) | static_cast<std::uint32_t>(m_RXPin);
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_7;
  LL_GPIO_Init(m_GPIOx, &GPIO_InitStruct);

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  USART_InitStruct.BaudRate = m_BaudRate;
  USART_InitStruct.DataWidth = LL_USART_DATAWIDTH_8B;
  USART_InitStruct.StopBits = LL_USART_STOPBITS_1;
  USART_InitStruct.Parity = LL_USART_PARITY_NONE;
  USART_InitStruct.TransferDirection = LL_USART_DIRECTION_TX_RX;
  USART_InitStruct.HardwareFlowControl = LL_USART_HWCONTROL_NONE;
  USART_InitStruct.OverSampling = LL_USART_OVERSAMPLING_16;
  LL_USART_Init(m_USART, &USART_InitStruct);
  LL_USART_DisableIT_CTS(m_USART);
  LL_USART_ConfigAsyncMode(m_USART);
  LL_USART_Enable(m_USART);
}

void USART::SendBytes(const std::uint8_t* pData, std::size_t size) {
  while (!LL_USART_IsActiveFlag_TXE(m_USART)) {
  }

  for (std::size_t i = 0; i < size; ++i) {
    LL_USART_TransmitData8(m_USART, pData[i]);
    while (!LL_USART_IsActiveFlag_TXE(m_USART)) {
    }
  }
  while (!LL_USART_IsActiveFlag_TC(m_USART)) {
  }
}

std::size_t USART::ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                                std::chrono::milliseconds timeout) {
  std::size_t dataRead{0};
  std::uint32_t initTick = m_SysTimer.TickCount();
  std::chrono::microseconds usPerTick = m_SysTimer.MicroSecondsPerTick();

  for (size_t i = 0; i < maxSize; ++i) {
    std::chrono::microseconds waitTime{0};
    bool timeoutOccurred = false;
    while (!LL_USART_IsActiveFlag_RXNE(m_USART) && !timeoutOccurred) {
      waitTime = usPerTick * ISystemTimer::CalculateTickDelta(
                                 initTick, m_SysTimer.TickCount());
      timeoutOccurred = waitTime > timeout;
    }

    if (timeoutOccurred) {
      break;  // for
    }
    pData[i] = LL_USART_ReceiveData8(m_USART);
  }

  return dataRead;
}
}  // namespace openstm::hal::stmicro::f3
