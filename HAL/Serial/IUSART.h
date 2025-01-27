#pragma once

#include <chrono>
#include <cstdint>

#include "PinID.h"

namespace openstm::hal {

class IUSART {
 public:
  virtual void Initialize() = 0;

  virtual PinID TXPin() const = 0;
  virtual PinID RXPin() const = 0;
  virtual std::uint32_t BaudRate() const = 0;

  virtual void SendBytes(const std::uint8_t* pData, std::size_t size) = 0;
  virtual std::size_t ReceiveBytes(std::uint8_t* pData, std::size_t maxSize,
                                   std::chrono::milliseconds timeout) = 0;
};
}  // namespace openstm::hal
