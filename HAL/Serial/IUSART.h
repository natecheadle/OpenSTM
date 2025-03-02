#pragma once

#include <Event/Event.hpp>
#include <cstdint>
#include <functional>
#include <span>

#include "PinID.h"

namespace openstm::hal {

class IUSART {
 public:
  enum class ErrorCode {
    None = 0,
    Parity,
    ReceiveTimeout,
    Overrun,
    Framing,
    NoiseDetected,
    NotIdle,
    BufferFull,
    DMAError,

    LAST
  };

  struct Result {
    Result() : Error(ErrorCode::None), BytesTransmitted(0) {}
    Result(ErrorCode error) : Error(error), BytesTransmitted(0) {}
    Result(size_t bytesTransmitted)
        : Error(ErrorCode::None), BytesTransmitted(bytesTransmitted) {}
    Result(ErrorCode error, size_t bytesTransmitted)
        : Error(error), BytesTransmitted(bytesTransmitted) {}

    ErrorCode Error;
    size_t BytesTransmitted;
  };

  using BufferFullEvent = lib::Event<2, IUSART&>;
  using BufferFullSub = lib::Event<2, IUSART&>::Subscription;

  virtual void Initialize() = 0;

  virtual PinID TXPin() const = 0;
  virtual PinID RXPin() const = 0;
  virtual std::uint32_t BaudRate() const = 0;

  virtual size_t BufferedRxBytes() const = 0;
  virtual size_t BufferedTxMessages() const = 0;

  virtual void EnableError(ErrorCode error) = 0;
  virtual void EnableAllErrors() = 0;
  virtual void DisableError(ErrorCode error) = 0;
  virtual void DisableAllErrors() = 0;
  virtual bool IsErrorEnabled(ErrorCode error) = 0;

  virtual void FlushRxBuffer() = 0;
  virtual void FlushTxBuffer() = 0;

  virtual bool IsRxIdle() const = 0;

  virtual Result SendBytes(std::span<const std::uint8_t> data) = 0;

  virtual void SendBytesAsync(
      std::span<const std::uint8_t> data,
      std::function<void(const Result&)> completionCallback) = 0;
  virtual Result ReceiveBytes(std::span<std::uint8_t> buffer,
                              std::uint32_t timeout) = 0;
  virtual void ReceiveBytesAsync(
      std::span<std::uint8_t> buffer, std::uint32_t timeout,
      std::function<void(const Result&)> completionCallback) = 0;

  virtual BufferFullSub SubscribeBufferFull(
      std::function<void(IUSART&)> callback) = 0;
};
}  // namespace openstm::hal
