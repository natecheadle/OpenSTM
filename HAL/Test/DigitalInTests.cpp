#include <Vendor/Desktop/IO/DigitalIn.h>
#include <gtest/gtest.h>

namespace openstm::hal::desktop::test {
class DigitalInFixture : public testing::Test {
 public:
  DigitalIn DigitalInput{PinID::Four};

	 DigitalInFixture() { DigitalInput.Initialize();
  }
};

TEST_F(DigitalInFixture, ValidateInitialState) {
  ASSERT_EQ(DigitalInput.ID(), PinID::Four);
  ASSERT_EQ(DigitalInput.GetState(), DigitalState::LOW);
}

TEST_F(DigitalInFixture, ValidateGetState) {
  DigitalInput.SetState(DigitalState::HIGH);
  ASSERT_EQ(DigitalInput.GetState(), DigitalState::HIGH);
  DigitalInput.SetState(DigitalState::LOW);
  ASSERT_EQ(DigitalInput.GetState(), DigitalState::LOW);
}

TEST_F(DigitalInFixture, ValidateStateChangedCallback) {
  DigitalState currentState{DigitalState::LOW};
  auto sub = DigitalInput.AttachToInterrupt(
      [&](DigitalState state) { currentState = state; });
  DigitalInput.SetState(DigitalState::HIGH);

  ASSERT_EQ(currentState, DigitalState::HIGH);
  ASSERT_EQ(DigitalInput.GetState(), DigitalState::HIGH);
}

}
