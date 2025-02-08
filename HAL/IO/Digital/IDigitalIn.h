#pragma once

#include <Event/Event.hpp>
#include <functional>

#include "IDigitalIO.h"

#ifndef MAX_CALLBACKS_PER_PIN
#define MAX_CALLBACKS_PER_PIN 1
#endif

namespace openstm::hal {

class IDigitalIn : public IDigitalIO {
 public:
  using StateChangedEvent = lib::Event<MAX_CALLBACKS_PER_PIN, DigitalState>;
  using StateChangedSub = StateChangedEvent::Subscription;

  virtual StateChangedSub AttachToInterrupt(
      std::function<void(DigitalState)> f) = 0;
};
}  // namespace openstm::hal
