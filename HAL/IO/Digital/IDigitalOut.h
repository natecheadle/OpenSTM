#pragma once

#include "DigitalState.h"
#include "IDigitalIO.h"

namespace openstm::hal {

class IDigitalOut : public IDigitalIO {
 public:
  virtual void Toggle() = 0;

  virtual void SetState(DigitalState state) = 0;
};
}  // namespace openstm::hal
