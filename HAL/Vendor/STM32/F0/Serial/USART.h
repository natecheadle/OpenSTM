#pragma once

#include <Container/RingBuffer.hpp>

#include "Serial/USART_Base.h"
#include "stm32f0xx.h"

namespace openstm::hal::stmicro::f0 {

class USART : public USART_Base {
  struct DMAMasks {
    DMAMasks(USART_TypeDef* usart);
    std::uint32_t TXChannel;
    std::uint32_t RXChannel;

    std::uint32_t TXGI;
    std::uint32_t TXTC;
    std::uint32_t TXHT;
    std::uint32_t TXTE;

    std::uint32_t TXClearGI;
    std::uint32_t TXClearTC;
    std::uint32_t TXClearHT;
    std::uint32_t TXClearTE;

    std::uint32_t RXGI;
    std::uint32_t RXTC;
    std::uint32_t RXHT;
    std::uint32_t RXTE;

    std::uint32_t RXClearGI;
    std::uint32_t RXClearTC;
    std::uint32_t RXClearHT;
    std::uint32_t RXClearTE;
  };

  GPIO_TypeDef* const m_GPIOx;
  USART_TypeDef* const m_USART;
  const bool m_TxDMAEnabled;
  const bool m_RxDMAEnabled;
  const DMAMasks m_DMAMask;

  struct ReceiveMessage {
    std::span<std::uint8_t> Buffer;
    size_t ReceivedSize;
    std::function<void(const Result&)> Callback;
  };

  ReceiveMessage* m_pActiveReceive{nullptr};
  ReceiveMessage m_ActiveReceive;

 public:
  USART(PinID txPin, PinID rxPin, GPIO_TypeDef* gpiox, USART_TypeDef* usart,
        std::uint32_t baudRate, size_t txBufferSize, size_t rxBufferSize,
        bool txDMAEnabled, bool rxDMAEnabled);
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

  void HandleUSARTInterrupt();
  void HandleDMATxInterrupt();
  void HandleDMARxInterrupt();

 private:
  void DisableDMAChannel(std::uint32_t channel);
  void SetTXBuffer(std::span<const std::uint8_t> buffer);
  void ResetRxDMABuffer(std::span<std::uint8_t> buffer);

  void HandleReceiveError(ErrorCode error);
  void ReceiveByte(std::uint8_t val);
  void ReceiveEnable();
  void TransmitDataEmpty();
  void TransmissionComplete();
  void ReadDataNotEmpty();
};
}  // namespace openstm::hal::stmicro::f0
