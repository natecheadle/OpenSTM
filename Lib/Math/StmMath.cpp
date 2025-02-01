#include "StmMath.h"

namespace openstm::lib::math {

void AddArr(std::span<double> in1, std::span<double> in2,
            std::span<double> out) {
  size_t size = in1.size();
  if (in2.size() < size) {
    size = in2.size();
  }
  if (out.size() < size) {
    size = out.size();
  }
  for (size_t i = 0; i < size; ++i) {
    out[i] = in1[i] + in2[i];
  }
}
}  // namespace openstm::lib::math
