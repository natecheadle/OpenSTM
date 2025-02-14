#pragma once

#include <Container/RingBuffer.hpp>
#include <bitset>

#include "Serial/IUSART.h"
#include "stm32f303xe.h"

#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 32
#endif

namespace openstm::hal::stmicro::f3 {

class USART : public IUSART {
  GPIO_TypeDef* const m_GPIOx;
  USART_TypeDef* const m_USART;
  const PinID m_TXPin;
  const PinID m_RXPin;
  const std::uint32_t m_BaudRate;

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

  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_TxBuffer;
  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_RxBuffer;

  SendMessage* m_pActiveSend{nullptr};
  SendMessage m_ActiveSend;
  ReceiveMessage* m_pActiveReceive{nullptr};
  ReceiveMessage m_ActiveReceive;

  std::bitset<(size_t)ErrorCode::LAST> m_EnabledErrors;

 public:
  USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox, USART_TypeDef* usart,
        std::uint32_t baudRate);
  USART(const USART& other) = delete;
  USART(USART&& other);

  ~USART() = default;

  void Initialize() override;

  PinID TXPin() const override;
  PinID RXPin() const override;
  std::uint32_t BaudRate() const override;
  size_t BufferedRxBytes() const override;
  size_t BufferedTxBytes() const override;
  bool IsRxIdle() const override;
  bool IsTxIdle() const override;

  void EnableError(ErrorCode error) override;
  void EnableAllErrors() override;
  void DisableError(ErrorCode error) override;
  void DisableAllErrors() override;
  bool IsErrorEnabled(ErrorCode error) override;

  void FlushRxBuffer() override;
  void FlushTxBuffer() override;

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
