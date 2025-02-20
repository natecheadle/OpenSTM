#include "USART.h"

#include <atomic>
#include <future>

namespace openstm::hal::desktop {

USART::USART(PinID txPin, PinID rxPin, std::uint32_t baudRate)
    : USART_Base(txPin, rxPin, baudRate),
      m_BackgroundTimer(m_BackgroundContext) {}

USART::USART(USART&& other)
    : USART_Base(std::forward<USART_Base>(other)),
      m_BackgroundTimer(m_BackgroundContext) {}

USART::~USART() {
  m_BackgroundContext.stop();
  m_SerialThread.join();
}

void USART::Initialize() {
  m_PerByteTime = boost::asio::chrono::microseconds{
      static_cast<long long>(1.0 / BaudRate() * 1e6)};
  m_SerialThread = std::thread([this]() {
    m_BackgroundTimer.expires_from_now(m_PerByteTime);
    m_BackgroundTimer.async_wait([this](const boost::system::error_code& ec) {
      SendReceiveNextByte(ec);
    });
    m_BackgroundContext.run();
  });
}

bool USART::IsRxIdle() const {
  return !m_RxFuture.valid() || m_RxFuture.wait_for(std::chrono::seconds(0)) ==
                                    std::future_status::ready;
}

bool USART::IsTxIdle() const {
  return !m_TxFuture.valid() || m_TxFuture.wait_for(std::chrono::seconds(0)) ==
                                    std::future_status::ready;
}

IUSART::Result USART::SendBytes(const std::uint8_t* pData, std::size_t size) {
  Result rslt{};
  SendBytesAsync(pData, size, [&](const Result& completionResult) {
    rslt = completionResult;
  });
  m_TxFuture.get();
  return rslt;
}

void USART::SendBytesAsync(
    const std::uint8_t* pData, std::size_t size,
    std::function<void(const Result&)> completionCallback) {
  std::size_t bufferedCount{0};
  for (size_t i = 0; i < size; ++i) {
    if (TxBuffer().Push(pData[i])) {
      bufferedCount++;
    }
  }
  m_TxFuture = std::async(std::launch::async, [pData, size, completionCallback,
                                               bufferedCount, this]() {
    std::atomic<std::size_t> sentCount{0};
    auto subscription =
        SubscribeSentByte([&](std::uint8_t sentByte) { sentCount++; });

    while (sentCount < bufferedCount) {
      std::this_thread::yield();
    }
    completionCallback({sentCount});
  });
}

IUSART::Result USART::ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                                   std::uint32_t timeout) {
  Result rslt{};
  ReceiveBytesAsync(
      pData, maxSize, timeout,
      [&](const Result& completionResult) { rslt = completionResult; });
  m_RxFuture.get();
  return rslt;
}

void USART::ReceiveBytesAsync(
    std::uint8_t* pData, std::size_t maxSize, std::uint32_t,
    std::function<void(const Result&)> completionCallback) {
  if (IsRxIdle()) {
    m_RxFuture = std::async(
        std::launch::async, [pData, maxSize, completionCallback, this]() {
          boost::asio::chrono::microseconds waitTime{
              static_cast<long long>(1.0 / BaudRate() * 1e6)};
          std::size_t receivedSize{0};

          while (receivedSize < maxSize) {
            std::uint8_t nextByte{0x00};
            if (RxBuffer().Pop(nextByte)) {
              pData[receivedSize] = nextByte;
              receivedSize++;
            } else {
              std::this_thread::yield();
            }
          }
          completionCallback(Result{receivedSize});
        });
  } else {
    completionCallback(Result{ErrorCode::NotIdle});
  }
}

void USART::InjectReceiveBytes(std::span<std::uint8_t> data) {
  for (std::uint8_t val : data) {
    m_ToReceiveBuffer.Push(val);
  }
}

lib::Event<2, std::uint8_t>::Subscription USART::SubscribeSentByte(
    std::function<void(std::uint8_t)> callback) {
  return m_TxByteSentEvent.Subscribe(std::move(callback));
}

void USART::SendReceiveNextByte(const boost ::system::error_code& ec) {
  if (ec) {
    return;
  }
  std::uint8_t nextByte{0x00};
  if (TxBuffer().Pop(nextByte)) {
    m_TxByteSentEvent.Invoke(nextByte);
  }
  if (m_ToReceiveBuffer.Pop(nextByte)) {
    RxBuffer().Push(nextByte);
  }
  m_BackgroundTimer.expires_at(m_BackgroundTimer.expiry() + m_PerByteTime);

  m_BackgroundTimer.async_wait(
      [this](const boost::system::error_code& ec) { SendReceiveNextByte(ec); });
}
}  // namespace openstm::hal::desktop
