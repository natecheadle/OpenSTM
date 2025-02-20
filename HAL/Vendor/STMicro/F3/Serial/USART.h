#pragma once

#include <Container/RingBuffer.hpp>
#include <bitset>

#include "Serial/USART_Base.h"
#include "stm32f303xe.h"

namespace openstm::hal::stmicro::f3 {

class USART : public USART_Base {
  GPIO_TypeDef* const m_GPIOx;
  USART_TypeDef* const m_USART;

  struct SendMessage {
    size_t Size;
    size_t Sent;
    std::function<void(const Result&)> Callback;
  };

  struct ReceiveMessage {
    std::uint8_t* Buffer;
    size_t BufferSize;
    size_t ReceivedSize;
    std::function<void(const Result&)> Callback;
  };

  SendMessage* m_pActiveSend{nullptr};
  SendMessage m_ActiveSend;
  ReceiveMessage* m_pActiveReceive{nullptr};
  ReceiveMessage m_ActiveReceive;

 public:
  USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox, USART_TypeDef* usart,
        std::uint32_t baudRate);
  USART(const USART& other) = delete;
  USART(USART&& other);

  ~USART() = default;

  void Initialize() override;

  bool IsRxIdle() const override;
  bool IsTxIdle() const override;

  Result SendBytes(const std::uint8_t* pData, std::size_t size) override;

  void SendBytesAsync(
      const std::uint8_t* pData, std::size_t size,
      std::function<void(const Result&)> completionCallback) override;

  Result ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                      std::uint32_t timeout) override;

  void ReceiveBytesAsync(
      std::uint8_t* pData, std::size_t maxSize, std::uint32_t timeout,
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
