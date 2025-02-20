#pragma once

#include <Event/Event.hpp>

#include "IO/Digital/IDigitalIn.h"
#include "PinID.h"

namespace openstm::hal::desktop {

class DigitalIn : public IDigitalIn {
  const PinID m_ID;
  DigitalState m_State;

  StateChangedEvent m_StateChangedEvent;

 public:
  DigitalIn(PinID id, DigitalState initialState = DigitalState::LOW);

  DigitalIn(const DigitalIn& other) = delete;
  DigitalIn& operator=(const DigitalIn& other) = delete;

  DigitalIn(DigitalIn&& other) noexcept = default;
  DigitalIn& operator=(DigitalIn&& other) noexcept = delete;

  ~DigitalIn() = default;

  PinID ID() const override;
  void Initialize() override;

  DigitalState GetState() const override;

  StateChangedSub AttachToInterrupt(
      std::function<void(DigitalState)> f) override;

  void SetState(DigitalState state);
};
}  // namespace openstm::hal::desktop
