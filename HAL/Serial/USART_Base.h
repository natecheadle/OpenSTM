#pragma once

#include <Container/RingBuffer.hpp>
#include <bitset>

#include "Serial/IUSART.h"

#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 32
#endif

namespace openstm::hal {

class USART_Base : public IUSART {
  const PinID m_TXPin;
  const PinID m_RXPin;
  const std::uint32_t m_BaudRate;

  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_TxBuffer;
  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_RxBuffer;

  std::bitset<(size_t)ErrorCode::LAST> m_EnabledErrors;

 public:
  USART_Base(PinID txPin, PinID rxPin, std::uint32_t baudRate);
  USART_Base(const USART_Base& other) = delete;
  USART_Base(USART_Base&& other) = default;

  ~USART_Base() = default;

  PinID TXPin() const override;
  PinID RXPin() const override;
  std::uint32_t BaudRate() const override;
  size_t BufferedRxBytes() const override;
  size_t BufferedTxBytes() const override;

  void EnableError(ErrorCode error) override;
  void EnableAllErrors() override;
  void DisableError(ErrorCode error) override;
  void DisableAllErrors() override;
  bool IsErrorEnabled(ErrorCode error) override;

  void FlushRxBuffer() override;
  void FlushTxBuffer() override;

 protected:
  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& TxBuffer();
  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& RxBuffer();
  
  const lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& TxBuffer()const ;
  const lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& RxBuffer()const;

};
}  // namespace openstm::hal
