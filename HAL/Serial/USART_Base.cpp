#include "USART_Base.h"

namespace openstm::hal {

USART_Base::USART_Base(PinID txPin, PinID rxPin, std::uint32_t baudRate,
                       size_t txBufferSize, size_t rxBufferSize)
    : m_TXPin(txPin),
      m_RXPin(rxPin),
      m_BaudRate(baudRate),
      m_TxBuffer(txBufferSize),
      m_RxBuffer(rxBufferSize) {}

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

IUSART::BufferFullSub USART_Base::SubscribeBufferFull(
    std::function<void(IUSART&)> callback) {
  return m_BufferFullEvent.Subscribe(std::move(callback));
}

size_t USART_Base::RxBufferCount() const { return m_RxBuffer.BufferedCount(); }

bool USART_Base::PopTxBuffer(SendMessage& nextMsg) {
  return m_TxBuffer.Pop(nextMsg);
}

bool USART_Base::PopTxBuffer(std::uint8_t& nextByte) {
  if (m_TxBuffer.IsEmpty()) {
    return false;
  }
  SendMessage* msg;
  if (m_TxBuffer.Next(msg) && msg->SentData < msg->Data.size()) {
    nextByte = msg->Data[msg->SentData];
    msg->SentData++;
    return true;
  }
  return false;
}

bool USART_Base::PopRxBuffer(std::uint8_t& nextByte) {
  return m_RxBuffer.Pop(nextByte);
}

bool USART_Base::PushTxBuffer(const SendMessage& nextMessage) {
  return m_TxBuffer.Push(nextMessage);
}

bool USART_Base::PushRxBuffer(std::uint8_t nextByte) {
  if (m_RxBuffer.Push(nextByte)) {
    return true;
  }

  m_BufferFullEvent.Invoke(*this);
  return false;
}

}  // namespace openstm::hal
