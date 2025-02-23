#pragma once

#include <Container/RingBuffer.hpp>

#include "Serial/USART_Base.h"
#include "stm32f303xe.h"

namespace openstm::hal::stmicro::f3 {

class USART : public USART_Base {
  GPIO_TypeDef* const m_GPIOx;
  USART_TypeDef* const m_USART;

  struct ReceiveMessage {
    std::span<std::uint8_t> Buffer;
    size_t ReceivedSize;
    std::function<void(const Result&)> Callback;
  };

  ReceiveMessage* m_pActiveReceive{nullptr};
  ReceiveMessage m_ActiveReceive;

 public:
  USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox, USART_TypeDef* usart,
        std::uint32_t baudRate, size_t txBufferSize, size_t rxBufferSize);
  USART(const USART& other) = delete;
  USART(USART&& other);

  ~USART() = default;

  void Initialize() override;

  bool IsRxIdle() const override;

  Result SendBytes(std::span<const std::uint8_t> data) override;

  void SendBytesAsync(
      std::span<const std::uint8_t> data,
      std::function<void(const Result&)> completionCallback) override;

  Result ReceiveBytes(std::span<std::uint8_t> buffer,
                      std::uint32_t timeout) override;

  void ReceiveBytesAsync(
      std::span<std::uint8_t> buffer, std::uint32_t timeout,
      std::function<void(const Result&)> completionCallback) override;

  void HandleInterrupt();

 private:
  void HandleReceiveError(ErrorCode error);
  void ReceiveByte(std::uint8_t val);
  void ReceiveEnable();
  void TransmitDataEmpty();
  void TransmissionComplete();
  void ReadDataNotEmpty();
};
}  // namespace openstm::hal::stmicro::f3
