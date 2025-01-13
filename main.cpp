#include <App.hpp>

#include "Boards/Nucleo64-F303RE/App.h"
#include "stm32f3xx_ll_utils.h"

void Update();

namespace board = openstm::hal::boards::nucleo64f303re;
openstm::hal::AppWrapper<board::App> App(board::App([]() { Update(); }));

int main() { App.Run(); }

void Update() {
  LL_mDelay(500);
  board::App::LED2.Toggle();
}