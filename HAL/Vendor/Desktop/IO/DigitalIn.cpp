#include "DigitalIn.h"

namespace openstm::hal::desktop {
DigitalIn::DigitalIn(PinID id, DigitalState initialState)
    : m_ID(id), m_State(initialState) {}

PinID DigitalIn::ID() const { return m_ID; }

void DigitalIn::Initialize() {}

DigitalState DigitalIn::GetState() const { return m_State; }

IDigitalIn::StateChangedSub DigitalIn::AttachToInterrupt(
    std::function<void(DigitalState)> f) {
  return m_StateChangedEvent.Subscribe(std::move(f));
}

void DigitalIn::SetState(DigitalState state) {
  if (m_State != state) {
    m_State = state;
    m_StateChangedEvent.Invoke(m_State);
  }
}

}  // namespace openstm::hal::desktop
