#pragma once

#include <span>

namespace openstm::lib::math {
void AddArr(std::span<double> in1, std::span<double> in2,
            std::span<double> out);
}  // namespace openstm::lib::math
