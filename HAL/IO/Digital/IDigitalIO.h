#pragma once

#include "DigitalState.h"
#include "PinID.h"

namespace openstm::hal {

class IDigitalIO {
 public:
  virtual PinID ID() const = 0;

  virtual void Initialize() = 0;
  virtual DigitalState GetState() const = 0;
};
}  // namespace openstm::hal
