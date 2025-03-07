#pragma once

#include <type_traits>

#include "Driver.hpp"
#include "IUSART.h"

namespace openstm::hal {

template <typename T>
class USART : public IUSART, public Driver<T> {
  static_assert(std::is_base_of_v<IUSART, T>, "T must implement IUSART");

 public:
  USART(T&& uart) : Driver<T>(std::forward<T>(uart)) {}
  ~USART() = default;

  void Initialize() override { Driver<T>::Device().Initialize(); }

  PinID TXPin() const override { return Driver<T>::Device().TXPin(); }

  PinID RXPin() const override { return Driver<T>::Device().RXPin(); }

  std::uint32_t BaudRate() const override {
    return Driver<T>::Device().BaudRate();
  }

  size_t BufferedRxBytes() const override {
    return Driver<T>::Device().BufferedRxBytes();
  }

  size_t BufferedTxMessages() const override {
    return Driver<T>::Device().BufferedTxMessages();
  }

  void EnableError(ErrorCode error) override {
    Driver<T>::Device().EnableError(error);
  }

  void EnableAllErrors() override { Driver<T>::Device().EnableAllErrors(); }

  void DisableError(ErrorCode error) override {
    Driver<T>::Device().DisableError(error);
  }

  void DisableAllErrors() override { Driver<T>::Device().DisableAllErrors(); }

  bool IsErrorEnabled(ErrorCode error) override {
    return Driver<T>::Device().IsErrorEnabled(error);
  }

  void FlushRxBuffer() override { return Driver<T>::Device().FlushRxBuffer(); }

  void FlushTxBuffer() override { return Driver<T>::Device().FlushTxBuffer(); }

  bool IsRxIdle() const override { return Driver<T>::Device().IsRxIdle(); }

  Result SendBytes(std::span<const std::uint8_t> data) override {
    return Driver<T>::Device().SendBytes(data);
  }

  void SendBytesAsync(
      std::span<const std::uint8_t> data,
      std::function<void(const Result&)> completionCallback) override {
    Driver<T>::Device().SendBytesAsync(data, std::move(completionCallback));
  }

  Result ReceiveBytes(std::span<std::uint8_t> buffer,
                      std::uint32_t timeout) override {
    return Driver<T>::Device().ReceiveBytes(buffer, timeout);
  }

  void ReceiveBytesAsync(
      std::span<std::uint8_t> buffer, std::uint32_t timeout,
      std::function<void(const Result&)> completionCallback) override {
    return Driver<T>::Device().ReceiveBytesAsync(buffer, timeout,
                                                 std::move(completionCallback));
  }

  BufferFullSub SubscribeBufferFull(
      std::function<void(IUSART&)> callback) override {
    return Driver<T>::Device().SubscribeBufferFull(std::move(callback));
  }
};
}  // namespace openstm::hal
