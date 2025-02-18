#pragma once

#include <Event/Event.hpp>
#include <Container/RingBuffer.hpp>
#include <bitset>
#include <span>
#include <boost/asio.hpp>
#include <thread>

#include "Serial/USART_Base.h"

#ifndef USART_BUFFER_SIZE
#define USART_BUFFER_SIZE 32
#endif

namespace openstm::hal::desktop {

class USART : public USART_Base {
  boost::asio::io_context m_TxContext;
  boost::asio::io_context m_RxContext;

  boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
      m_TxWorkGuard;

  boost::asio::executor_work_guard<boost::asio::io_context::executor_type>
      m_RxWorkGuard;

  std::jthread m_TxThread;
  std::jthread m_RxThread;

  std::future<void> m_TxFuture;
  std::future<void> m_RxFuture;

  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_ToReceiveBuffer;

  lib::Event<2, std::uint8_t> m_TxByteSentEvent;

 public:
  USART(PinID txPin, PinID rxPin, std::uint32_t baudRate);
  USART(const USART& other) = delete;
  USART(USART&& other) = default;

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

  void InjectReceiveBytes(std::span<std::uint8_t> data);
  lib::Event<2, std::uint8_t>::Subscription SubscribeSentByte(
      std::function<void(std::uint8_t)> callback);

};
}  // namespace openstm::hal::desktop
