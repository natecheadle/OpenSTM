#include <App.hpp>
#include <algorithm>
#include <array>
#include <string_view>

#include "Boards/Nucleo64-F303RE/App.h"

std::array<std::uint8_t, 64> BUFFER;
std::string_view NOTHING = "NOTHING RECEIVED\n";

void Update();

namespace board = openstm::hal::boards::nucleo64f303re;
openstm::hal::AppWrapper<board::App> App(board::App([]() { Update(); }));

int main() {
  auto sub = board::App::Button1.AttachToInterrupt(
      [](openstm::hal::DigitalState) { board::App::LED2.Toggle(); });
  App.Run();
}

void Update() {
  BUFFER.fill(0x00);
  board::App::USB_USART.ReceiveBytes(BUFFER.data(), BUFFER.size(),
                                     std::chrono::seconds(1));

  auto end = std::find(BUFFER.begin(), BUFFER.end(), 0x00);
  size_t strLen = std ::distance(BUFFER.begin(), end);
  if (strLen == 0) {
    board::App::USB_USART.SendBytes(
        reinterpret_cast<const std::uint8_t*>(NOTHING.data()), NOTHING.size());
  } else {
    board::App::USB_USART.SendBytes(BUFFER.data(), strLen);
  }
}
