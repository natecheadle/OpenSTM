#include "App.h"

namespace openstm::hal {

void App::Run() {
  Initialize();
  while (true) {
    Update();
  }
}

} // namespace openstm::hal