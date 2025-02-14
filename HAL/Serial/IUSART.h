#pragma once

#include <cstdint>
#include <functional>

#include "PinID.h"

namespace openstm::hal {

class IUSART {
 public:
  enum class ErrorCode {
    None = 0,
    Parity = 1,
    ReceiveTimeout = 2,
    Overrun = 3,
    Framing = 4,
    NoiseDetected = 5,
    NotIdle = 6,

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

  virtual void Initialize() = 0;

  virtual PinID TXPin() const = 0;
  virtual PinID RXPin() const = 0;
  virtual std::uint32_t BaudRate() const = 0;

  virtual size_t BufferedRxBytes() const = 0;
  virtual size_t BufferedTxBytes() const = 0;

  virtual void EnableError(ErrorCode error) = 0;
  virtual void EnableAllErrors() = 0;
  virtual void DisableError(ErrorCode error) = 0;
  virtual void DisableAllErrors() = 0;
  virtual bool IsErrorEnabled(ErrorCode error) = 0;

  virtual void FlushRxBuffer() = 0;
  virtual void FlushTxBuffer() = 0;

  virtual bool IsRxIdle() const = 0;
  virtual bool IsTxIdle() const = 0;

  virtual Result SendBytes(const std::uint8_t* pData, std::size_t size) = 0;
  virtual void SendBytesAsync(
      const std::uint8_t* pData, std::size_t size,
      std::function<void(const Result&)> completionCallback) = 0;
  virtual Result ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                              std::uint32_t timeout) = 0;
  virtual void ReceiveBytesAsync(
      std::uint8_t* pData, std::size_t maxSize, std::uint32_t timeout,
      std::function<void(const Result&)> completionCallback) = 0;
};
}  // namespace openstm::hal
