#include "USART_Base.h"

namespace openstm::hal {

USART_Base::USART_Base(PinID txPin, PinID rxPin, std::uint32_t baudRate)
    : m_TXPin(txPin), m_RXPin(rxPin), m_BaudRate(baudRate) {}

PinID USART_Base::TXPin() const { return m_TXPin; }

PinID USART_Base::RXPin() const { return m_RXPin; }

std::uint32_t USART_Base::BaudRate() const { return m_BaudRate; }

size_t USART_Base::BufferedRxBytes() const {
  return m_RxBuffer.BufferedCount();
}

size_t USART_Base::BufferedTxBytes() const {
  return m_TxBuffer.BufferedCount();
}

void USART_Base::EnableError(ErrorCode error) {
  m_EnabledErrors[(size_t)error] = true;
}

void USART_Base::EnableAllErrors() { m_EnabledErrors.set(); }

void USART_Base::DisableError(ErrorCode error) {
  m_EnabledErrors[(size_t)error] = false;
}

void USART_Base::DisableAllErrors() { m_EnabledErrors.reset(); }

bool USART_Base::IsErrorEnabled(ErrorCode error) {
  return m_EnabledErrors[(size_t)error];
}

void USART_Base::FlushRxBuffer() { m_RxBuffer.Clear(); }

void USART_Base::FlushTxBuffer() { m_TxBuffer.Clear(); }

lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& USART_Base::TxBuffer() {
  return m_TxBuffer;
}

lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& USART_Base::RxBuffer() {
  return m_RxBuffer;
}

const lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& USART_Base::TxBuffer()
    const {
  return m_TxBuffer;
}

const lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE>& USART_Base::RxBuffer()
    const {
  return m_RxBuffer;
}

}  // namespace openstm::hal
