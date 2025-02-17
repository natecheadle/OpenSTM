#include "DigitalOut.h"

namespace openstm::hal::desktop {

DigitalOut::DigitalOut(PinID id, DigitalState initialState)
    : m_ID(id), m_State(initialState) {}

PinID DigitalOut::ID() const { return m_ID; }

void DigitalOut::Initialize() {}

void DigitalOut::Toggle() {
  if (m_State == DigitalState::LOW) {
    m_State = DigitalState::HIGH;
  } else {
    m_State = DigitalState::LOW;
  }
}

DigitalState DigitalOut::GetState() const { return m_State; }

void DigitalOut::SetState(DigitalState state) { m_State = state; }
}  // namespace openstm::hal::desktop
