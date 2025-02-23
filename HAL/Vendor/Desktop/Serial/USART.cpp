#include "USART.h"

namespace openstm::hal::desktop {

USART::USART(PinID txPin, PinID rxPin, std::uint32_t baudRate,
             size_t txBufferSize, size_t rxBufferSize)
    : USART_Base(txPin, rxPin, baudRate, txBufferSize, rxBufferSize),
      m_BackgroundTimer(m_BackgroundContext),
      m_ToReceiveBuffer(rxBufferSize) {}

USART::USART(USART&& other)
    : USART_Base(std::forward<USART_Base>(other)),
      m_BackgroundTimer(m_BackgroundContext),
      m_ToReceiveBuffer(std::move(other.m_ToReceiveBuffer)) {}

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

IUSART::Result USART::SendBytes(std::span<const std::uint8_t> data) {
  std::atomic<bool> isComplete{false};
  Result rslt{};
  SendBytesAsync(data, [&](const Result& completionResult) {
    isComplete = true;
    rslt = completionResult;
  });
  while (!isComplete) {
    std::this_thread::yield();
  }

  return rslt;
}

void USART::SendBytesAsync(
    std::span<const std::uint8_t> data,
    std::function<void(const Result&)> completionCallback) {
  SendMessage msg{data, std::move(completionCallback)};
  if (!PushTxBuffer(msg)) {
    if (msg.CompletionCallback) {
      msg.CompletionCallback({ErrorCode::BufferFull});
    }
  }
}

IUSART::Result USART::ReceiveBytes(std::span<std::uint8_t> buffer,
                                   std::uint32_t timeout) {
  Result rslt{};
  ReceiveBytesAsync(buffer, timeout, [&](const Result& completionResult) {
    rslt = completionResult;
  });
  m_RxFuture.get();
  return rslt;
}

void USART::ReceiveBytesAsync(
    std::span<std::uint8_t> buffer, std::uint32_t,
    std::function<void(const Result&)> completionCallback) {
  if (IsRxIdle()) {
    m_RxFuture =
        std::async(std::launch::async, [buffer, completionCallback, this]() {
          boost::asio::chrono::microseconds waitTime{
              static_cast<long long>(1.0 / BaudRate() * 1e6)};
          std::size_t receivedSize{0};

          while (receivedSize < buffer.size()) {
            std::uint8_t nextByte{0x00};
            if (PopRxBuffer(nextByte)) {
              buffer[receivedSize] = nextByte;
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
  if (PopTxBuffer(nextByte)) {
    m_TxByteSentEvent.Invoke(nextByte);
  } else {
    SendMessage nextMsg;
    if (PopTxBuffer(nextMsg)) {
      assert(nextMsg.Data.size() == nextMsg.SentData);
      if (nextMsg.CompletionCallback) {
        nextMsg.CompletionCallback({nextMsg.SentData});
      }
    }
  }

  if (m_ToReceiveBuffer.Pop(nextByte)) {
    PushRxBuffer(nextByte);
  }
  m_BackgroundTimer.expires_at(m_BackgroundTimer.expiry() + m_PerByteTime);

  m_BackgroundTimer.async_wait(
      [this](const boost::system::error_code& ec) { SendReceiveNextByte(ec); });
}
}  // namespace openstm::hal::desktop
