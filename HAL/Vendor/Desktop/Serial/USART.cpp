#include "USART.h"

#include <atomic>
#include <future>

namespace openstm::hal::desktop {

USART::USART(PinID txPin, PinID rxPin, std::uint32_t baudRate)
    : USART_Base(txPin, rxPin, baudRate),
      m_TxWorkGuard(boost::asio::make_work_guard(m_TxContext)),
      m_RxWorkGuard(boost::asio::make_work_guard(m_RxContext)) {}

USART::USART(USART&& other)
    : USART_Base(std::forward<USART_Base>(other)),
      m_TxWorkGuard(boost::asio::make_work_guard(m_TxContext)),
      m_RxWorkGuard(boost::asio::make_work_guard(m_RxContext)) {}

USART::~USART() {
  m_TxContext.stop();
  m_RxContext.stop();
  m_TxThread.join();
  m_RxThread.join();
}

void USART::Initialize() {
  m_TxThread = std::thread([this]() {
    boost::asio::chrono::microseconds waitTime{
        static_cast<long long>(1.0 / BaudRate() * 1e6)};
    boost::asio::steady_timer timer(m_TxContext, waitTime);
    timer.async_wait([this](const boost::system::error_code&) {
      std::uint8_t nextByte{0x00};
      if (TxBuffer().Pop(nextByte)) {
        m_TxByteSentEvent.Invoke(nextByte);
      }
    });
    m_TxContext.run();
  });

  m_RxThread = std::thread([this]() {
    boost::asio::chrono::microseconds waitTime{
        static_cast<long long>(1.0 / BaudRate() * 1e6)};
    boost::asio::steady_timer timer(m_TxContext, waitTime);
    timer.async_wait([this](const boost::system::error_code&) {
      std::uint8_t nextByte{0x00};
      if (m_ToReceiveBuffer.Pop(nextByte)) {
        RxBuffer().Push(nextByte);
      }
    });
    m_RxContext.run();
  });
}

bool USART::IsRxIdle() const { return m_RxFuture.valid(); }

bool USART::IsTxIdle() const { return m_TxFuture.valid(); }

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
  m_TxFuture =
      std::async(std::launch::async, [pData, size, completionCallback, this]() {
        std::size_t bufferedCount{0};
        std::atomic<std::size_t> sentCount{0};
        auto subscription =
            SubscribeSentByte([&](std::uint8_t sentByte) { sentCount++; });
        for (size_t i = 0; i < size; ++i) {
          if (TxBuffer().Push(sentCount)) {
            bufferedCount++;
          }
        }
        while (sentCount < bufferedCount) {
          std::this_thread::yield();
        }
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
    std::uint8_t* pData, std::size_t maxSize, std::uint32_t timeout,
    std::function<void(const Result&)> completionCallback) {
  if (IsRxIdle()) {
    m_RxFuture = std::async(std::launch::async, [pData, maxSize, timeout,
                                                 completionCallback, this]() {
      boost::asio::chrono::microseconds waitTime{
          static_cast<long long>(1.0 / BaudRate() * 1e6)};
      std::size_t receivedSize{0};

      auto timeoutTime = std::chrono::microseconds{timeout * waitTime.count()} +
                         std::chrono::steady_clock::now();
      while (receivedSize < maxSize) {
        if (std::chrono::steady_clock::now() < timeoutTime) {
          completionCallback(Result{ErrorCode::ReceiveTimeout, receivedSize});
        }
        std::uint8_t nextByte{0x00};
        if (RxBuffer().Pop(nextByte)) {
          pData[receivedSize] = nextByte;
          receivedSize++;
        }

        std::this_thread::yield();
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

}  // namespace openstm::hal::desktop
