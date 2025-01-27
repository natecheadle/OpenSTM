#pragma once

#include "../Timer/SystemTimer.h"
#include "Serial/IUSART.h"
#include "stm32f303xe.h"

namespace openstm::hal::stmicro::f3 {

class USART : public IUSART {
  GPIO_TypeDef* const m_GPIOx;
  USART_TypeDef* const m_USART;
  const PinID m_TXPin;
  const PinID m_RXPin;
  const std::uint32_t m_BaudRate;
  const SystemTimer& m_SysTimer;

 public:
  USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox, USART_TypeDef* usart,
        std::uint32_t baudRate, const SystemTimer& timer);
  USART(const USART& other) = default;

  ~USART() = default;

  void Initialize() override;

  PinID TXPin() const override;
  PinID RXPin() const override;
  std::uint32_t BaudRate() const override;

  void SendBytes(const std::uint8_t* pData, std::size_t size) override;
  std::size_t ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                           std::chrono::milliseconds timeout) override;
};
}  // namespace openstm::hal::stmicro::f3
