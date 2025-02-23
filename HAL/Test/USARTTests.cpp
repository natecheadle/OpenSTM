#include <Vendor/Desktop/Serial/USART.h>
#include <gtest/gtest.h>

namespace openstm::hal::desktop::test {
class USARTFixture : public testing::Test {
  openstm::lib::Event<2, std::uint8_t>::Subscription m_Sub;

 public:
  USART Serial{PinID::Three, PinID::Four, 9600, 4, 32};

  USARTFixture() {
    openstm::lib::StaticStackAllocator<STATIC_ALLOCATOR_SIZE>::Reset();

    Serial.Initialize();
    m_Sub =
        Serial.SubscribeSentByte([this](uint8_t val) { ByteReceived(val); });
  }
  std::vector<std::uint8_t> SentData;

  void ByteReceived(uint8_t val) { SentData.push_back(val); }
};

TEST_F(USARTFixture, ValidateInitialState) {
  ASSERT_EQ(Serial.TXPin(), PinID::Three);
  ASSERT_EQ(Serial.RXPin(), PinID::Four);
  ASSERT_EQ(Serial.BaudRate(), 9600);
  ASSERT_TRUE(Serial.IsRxIdle());
}

TEST_F(USARTFixture, ValidateSendBytes) {
  std::vector<std::uint8_t> toSendData;
  for (std::uint8_t i = 0; i < 16; i++) {
    toSendData.push_back(i);
  }
  Serial.SendBytes(toSendData);

  ASSERT_EQ(SentData, toSendData);
}

TEST_F(USARTFixture, ValidateSendBytesAsync) {
  std::vector<std::uint8_t> toSendData;
  for (std::uint8_t i = 0; i < 16; i++) {
    toSendData.push_back(i);
  }
  std::atomic<bool> completed{false};
  Serial.SendBytesAsync(toSendData,
                        [&](const IUSART::Result&) { completed = true; });
  while (!completed) {
    std::this_thread::yield();
  }
  ASSERT_EQ(SentData, toSendData);
}

TEST_F(USARTFixture, ValidateReceiveBytes) {
  std::vector<std::uint8_t> toReceiveData;
  for (std::uint8_t i = 0; i < 16; i++) {
    toReceiveData.push_back(i);
  }

  std::vector<std::uint8_t> receivedData;
  receivedData.resize(toReceiveData.size());

  Serial.InjectReceiveBytes(toReceiveData);
  Serial.ReceiveBytes(receivedData, 100);
  ASSERT_TRUE(Serial.IsRxIdle());
  ASSERT_EQ(toReceiveData, receivedData);
}

TEST_F(USARTFixture, ValidateReceiveBytesAsync) {
  std::vector<std::uint8_t> toReceiveData;
  for (std::uint8_t i = 0; i < 16; i++) {
    toReceiveData.push_back(i);
  }

  std::vector<std::uint8_t> receivedData;
  receivedData.resize(toReceiveData.size());
  std::atomic<bool> completed{false};

  Serial.InjectReceiveBytes(toReceiveData);
  Serial.ReceiveBytesAsync(receivedData, 100,
                           [&](const IUSART::Result&) { completed = true; });
  while (!completed) {
    std::this_thread::yield();
  }

  ASSERT_TRUE(Serial.IsRxIdle());
  ASSERT_EQ(toReceiveData, receivedData);
}
}  // namespace openstm::hal::desktop::test
