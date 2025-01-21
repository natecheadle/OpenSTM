#pragma once

#include <functional>

#include "IDigitalIO.h"

namespace openstm::hal {

class IDigitalIn : public IDigitalIO {
 public:
  virtual int AttachToInterrupt(std::function<void()> f) = 0;
  virtual void RemoveInterrupt(int id) = 0;
};
}  // namespace openstm::hal
