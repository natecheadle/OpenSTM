#pragma once

#include "IO/Digital/IDigitalOut.h"

namespace openstm::hal::desktop {

class DigitalOut : public IDigitalOut {
  const PinID m_ID;
  DigitalState m_State;

 public:
  DigitalOut(PinID id, DigitalState initialState = DigitalState::LOW);
  DigitalOut(const DigitalOut& other) = default;

  ~DigitalOut() = default;

  PinID ID() const override;
  void Initialize() override;

  void Toggle() override;

  DigitalState GetState() const override;
  void SetState(DigitalState state) override;
};
}  // namespace openstm::hal::desktop
