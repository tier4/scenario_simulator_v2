#include <gtest/gtest.h>

#include <arithmetic/floating_point/comparison.hpp>

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}

TEST(Comparison, isApproximatelyEqualTo_equal)
{
  const float a = 0.1;
  const float b = 0.10000000000000001;
  
  EXPECT_TRUE(math::arithmetic::isApproximatelyEqualTo(a, b));

  const double af = -1000.1;
  const double bf = -1000.10000000000000001;
  
  EXPECT_TRUE(math::arithmetic::isApproximatelyEqualTo(af, bf));
}

TEST(Comparison, isApproximatelyEqualTo_notEqual)
{
  const float a = 0.1;
  const float b = 0.11;
  
  EXPECT_FALSE(math::arithmetic::isApproximatelyEqualTo(a, b));

  const double af = -1000.1;
  const double bf = -1000.11;
  
  EXPECT_FALSE(math::arithmetic::isApproximatelyEqualTo(af, bf));
}

TEST(Comparison, isEssentiallyEqualTo_equal)
{
  const float a = 0.1;
  const float b = 0.10000000000000001;
  
  EXPECT_TRUE(math::arithmetic::isApproximatelyEqualTo(a, b));

  const double af = -1000.1;
  const double bf = -1000.10000000000000001;
  
  EXPECT_TRUE(math::arithmetic::isApproximatelyEqualTo(af, bf));
}

TEST(Comparison, isEssentiallyEqualTo_notEqual)
{
  const float a = 0.1;
  const float b = 0.1000001;
  
  EXPECT_FALSE(math::arithmetic::isEssentiallyEqualTo(a, b));

  const double af = -1000.1;
  const double bf = -1000.1000001;
  
  EXPECT_FALSE(math::arithmetic::isEssentiallyEqualTo(af, bf));
}

TEST(Comparison, isDefinitelyLessThan_lessTwo)
{
  const float a = 0.1;
  const float b = 0.2;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyLessThan(a, b));

  const double af = -1.2;
  const double bf = -1.1;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyLessThan(af, bf));
}

TEST(Comparison, isDefinitelyLessThan_lessThree)
{
  const float a = 0.1;
  const float b = 0.2;
  const float c = 0.3;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyLessThan(a, b, c));

  const double af = -1.3;
  const double bf = -1.2;
  const double cf = -1.1;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyLessThan(af, bf, cf));
}

TEST(Comparison, isDefinitelyLessThan_moreTwo)
{
  const float a = 0.2;
  const float b = 0.1;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(a, b));

  const double af = -1.1;
  const double bf = -1.2;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(af, bf));
}

TEST(Comparison, isDefinitelyLessThan_moreThree)
{
  const float a = 0.3;
  const float b = 0.2;
  const float c = 0.1;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(a, b, c));

  const double af = -1.1;
  const double bf = -1.2;
  const double cf = -1.3;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(af, bf, cf));
}

TEST(Comparison, isDefinitelyLessThan_partlyMoreThree)
{
  const float a = 0.1;
  const float b = 0.2;
  const float c = 0.1;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(a, b, c));

  const double af = -1.1;
  const double bf = -1.2;
  const double cf = -1.1;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyLessThan(af, bf, cf));
}

TEST(Comparison, isDefinitelyGreaterThan_greater)
{
  const float a = 0.2;
  const float b = 0.1;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyGreaterThan(a, b));

  const double af = -1.1;
  const double bf = -1.2;
  
  EXPECT_TRUE(math::arithmetic::isDefinitelyGreaterThan(af, bf));
}

TEST(Comparison, isDefinitelyGreaterThan_notGreater)
{
  const float a = 0.1;
  const float b = 0.2;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyGreaterThan(a, b));

  const double af = -1.2;
  const double bf = -1.1;
  
  EXPECT_FALSE(math::arithmetic::isDefinitelyGreaterThan(af, bf));
}