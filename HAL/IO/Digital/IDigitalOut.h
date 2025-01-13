#pragma once

#include "DigitalState.h"
#include "PinID.h"

namespace openstm::hal {

class IDigitalOut {
 public:
  virtual PinID ID() const = 0;
  virtual void Toggle() = 0;

  virtual DigitalState State() const = 0;
  virtual void State(DigitalState state) = 0;
};
}  // namespace openstm::hal
