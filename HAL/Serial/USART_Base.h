#pragma once

#include <Allocator/StaticStackAllocator.hpp>
#include <Container/RingBuffer.hpp>
#include <bitset>

#include "Serial/IUSART.h"

namespace openstm::hal {

class USART_Base : public IUSART {
 protected:
  struct SendMessage {
    SendMessage() = default;

    SendMessage(std::span<const std::uint8_t> data,
                std::function<void(const Result&)> callback)
        : Data(data), CompletionCallback(callback) {}

    std::span<const std::uint8_t> Data;
    std::function<void(const Result&)> CompletionCallback;

    size_t SentData{0};
  };

 private:
  const PinID m_TXPin;
  const PinID m_RXPin;
  const std::uint32_t m_BaudRate;

  BufferFullEvent m_BufferFullEvent;

  lib::RingBuffer<SendMessage, lib::StaticStackAllocatorT<
                                   SendMessage, STATIC_ALLOCATOR_SIZE>>
      m_TxBuffer;
  lib::RingBuffer<std::uint8_t, lib::StaticStackAllocatorT<
                                    std::uint8_t, STATIC_ALLOCATOR_SIZE>>
      m_RxBuffer;

  std::bitset<(size_t)ErrorCode::LAST> m_EnabledErrors;

 public:
  USART_Base(PinID txPin, PinID rxPin, std::uint32_t baudRate,
             size_t txBufferSize, size_t rxBufferSize);
  USART_Base(const USART_Base& other) = delete;
  USART_Base(USART_Base&& other) = default;

  ~USART_Base() = default;

  PinID TXPin() const override;
  PinID RXPin() const override;
  std::uint32_t BaudRate() const override;
  size_t BufferedRxBytes() const override;
  size_t BufferedTxMessages() const override;

  void EnableError(ErrorCode error) override;
  void EnableAllErrors() override;
  void DisableError(ErrorCode error) override;
  void DisableAllErrors() override;
  bool IsErrorEnabled(ErrorCode error) override;

  void FlushRxBuffer() override;
  void FlushTxBuffer() override;

  BufferFullSub SubscribeBufferFull(
      std::function<void(IUSART&)> callback) override;

 protected:
  size_t RxBufferCount() const;

  bool PopTxBuffer(std::uint8_t& nextByte);
  bool PopTxBuffer(SendMessage& nextMessage);
  bool PopRxBuffer(std::uint8_t& nextByte);

  bool PushTxBuffer(const SendMessage& nextMessage);
  bool PushRxBuffer(std::uint8_t nextByte);
};
}  // namespace openstm::hal
