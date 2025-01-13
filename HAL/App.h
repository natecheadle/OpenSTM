#pragma once

namespace openstm::hal {

class App {
public:
  void Run();

protected:
  virtual void Initialize() = 0;
  virtual void Update() = 0;
};
} // namespace openstm::hal