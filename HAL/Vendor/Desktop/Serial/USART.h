#pragma once

#include <Container/RingBuffer.hpp>
#include <Event/Event.hpp>
#include <boost/asio.hpp>
#include <span>
#include <thread>

#include "Serial/USART_Base.h"

namespace openstm::hal::desktop {

class USART : public USART_Base {
  boost::asio::io_context m_BackgroundContext;
  boost::asio::steady_timer m_BackgroundTimer;
  std::chrono::microseconds m_PerByteTime{0};

  std::thread m_SerialThread;

  std::future<void> m_TxFuture;
  std::future<void> m_RxFuture;

  lib::RingBuffer<std::uint8_t, USART_BUFFER_SIZE> m_ToReceiveBuffer;

  lib::Event<2, std::uint8_t> m_TxByteSentEvent;

 public:
  USART(PinID txPin, PinID rxPin, std::uint32_t baudRate);
  USART(const USART& other) = delete;
  USART(USART&& other);

  ~USART();

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

 private:
  void SendReceiveNextByte(const boost ::system::error_code& ec);
};
}  // namespace openstm::hal::desktop
