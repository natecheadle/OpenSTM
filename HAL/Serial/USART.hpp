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

  void SendBytes(const std::uint8_t* pData, std::size_t size) override {
    Driver<T>::Device().SendBytes(pData, size);
  }

  std::size_t ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                           std::chrono::milliseconds timeout) override {
    return Driver<T>::Device().ReceiveBytes(pData, maxSize, timeout);
  }

  T& ConcreteObj() { return Driver<T>::Device(); }
  const T& ConcreteObj() const { return Driver<T>::Device(); }
};
}  // namespace openstm::hal
