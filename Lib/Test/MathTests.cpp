#include <Math/StmMath.h>
#include <gtest/gtest.h>

#include <vector>

namespace openstm::lib::math::test {
TEST(MathFixture, ValidateAdd) {
  std::vector<double> in1{{1.0, 2.0, 4.0}};
  std::vector<double> out{{0.0, 0.0, 0.0}};
  std::vector<double> out_exp{{2.0, 4.0, 8.0}};

  AddArr(in1, in1, out);
  ASSERT_EQ(out, out_exp);
}
}  // namespace openstm::lib::math::test
