#include <Vendor/Desktop/IO/DigitalOut.h>
#include <gtest/gtest.h>

namespace openstm::hal::desktop::test {
class DigitalOutFixture : public testing::Test {
 public:
  DigitalOut DigitalOutput{PinID::Four};

	 DigitalOutFixture() { DigitalOutput.Initialize();
  }
};

TEST_F(DigitalOutFixture, ValidateInitialState) {
  ASSERT_EQ(DigitalOutput.ID(), PinID::Four);
  ASSERT_EQ(DigitalOutput.GetState(), DigitalState::LOW);
}

TEST_F(DigitalOutFixture, ValidateGetState) {
  DigitalOutput.SetState(DigitalState::HIGH);
  ASSERT_EQ(DigitalOutput.GetState(), DigitalState::HIGH);
  DigitalOutput.SetState(DigitalState::LOW);
  ASSERT_EQ(DigitalOutput.GetState(), DigitalState::LOW);
}

TEST_F(DigitalOutFixture, ValidateToggle) {
  ASSERT_EQ(DigitalOutput.GetState(), DigitalState::LOW);
  DigitalOutput.Toggle();
  ASSERT_EQ(DigitalOutput.GetState(), DigitalState::HIGH);
}

}
