#include <tools/tools.hh>

#include <gtest/gtest.h>

TEST(ToolsSuite, simple)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(2, 2)});

  EXPECT_FLOAT_EQ(b_spline.getValue(0), 0.0);
  EXPECT_FLOAT_EQ(b_spline.getValue(1), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getValue(2), 2.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(0), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(1), 1.0);
  EXPECT_LT(b_spline.getDerValue(2) - 1.0, 1E-7);
  EXPECT_FLOAT_EQ(b_spline.get2DerValue(0), 0.0);
  EXPECT_LT(b_spline.get2DerValue(2), 1E-7);
}

TEST(ToolsSuite, add_another_point)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(2, 2)});

  b_spline.addPoint(BSpline::Point(3, 3));

  EXPECT_FLOAT_EQ(b_spline.getValue(0), 0.0);
  EXPECT_FLOAT_EQ(b_spline.getValue(1), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getValue(2), 2.0);
  EXPECT_FLOAT_EQ(b_spline.getValue(3), 3.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(0), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(1), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(2), 1.0);
  EXPECT_FLOAT_EQ(b_spline.getDerValue(3), 1.0);
}

TEST(ToolsSuite, two_same_point)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(1, 1)});

  EXPECT_LT(fabs(b_spline.getValue(0)), 1E-5);
  EXPECT_FLOAT_EQ(b_spline.getValue(1), 1.0);
  EXPECT_LT(fabs(b_spline.getDerValue(1) - 1.0), 1E-5);
  EXPECT_LT(fabs(b_spline.getDerValue(1) - 1.0), 1E-5);
}

TEST(ToolsSuite, two_same_point2)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(0, 0),
        BSpline::Point(1, 1)});

  EXPECT_LT(fabs(b_spline.getValue(0)), 1E-5);
  EXPECT_FLOAT_EQ(fabs(b_spline.getValue(1)), 1.0);
  EXPECT_LT(fabs(b_spline.getDerValue(1) - 1.0), 1E-5);
  EXPECT_LT(fabs(b_spline.getDerValue(1) - 1.0), 1E-5);
}

TEST(ToolsSuite, erase)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(2, 2)});

  b_spline.addPoint(BSpline::Point(3, 3));

  EXPECT_TRUE(b_spline.isInRange(0));
  b_spline.erase(1);

  // 1 must exist
  EXPECT_FALSE(b_spline.isInRange(0));
  EXPECT_TRUE(b_spline.isInRange(1));
  EXPECT_LT(b_spline.getValue(1) - 1.0, 1E-5);
  EXPECT_LT(b_spline.getValue(2) - 2.0, 1E-5);
}

TEST(ToolsSuite, erase_upper)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(2, 2)});

  b_spline.addPoint(BSpline::Point(3,
                                        3));
  EXPECT_TRUE(b_spline.isInRange(3));
  b_spline.eraseUpper(2);

  // 2 must exist
  EXPECT_FALSE(b_spline.isInRange(3));
  EXPECT_TRUE(b_spline.isInRange(2));
  EXPECT_LT(b_spline.getValue(0), 1E-5);
  EXPECT_LT(b_spline.getValue(2) - 2.0, 1E-5);
}

TEST(ToolsSuite, round_off_values)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(0, 0),
        BSpline::Point(0.5, -4 + 4 * cos(M_PI * (0.5) / 2.0))},
    -2, 2, 10);

  for (int i = 1; i < 10; ++i)
  {
    b_spline.addPoint(BSpline::Point(0.5 + i / 2.0,
                                          -4 + 4 * cos(M_PI * (0.5 + i) / 2.0)));
  }

  EXPECT_LT(b_spline.getValue(0), 1E-5);

  for (int i = 0; i < 10; ++i)
  {
    auto value = b_spline.getValue(0.5 + i / 2.0);
    EXPECT_GT(value, -2) << "In iteration: " << i;
    EXPECT_LT(value, 2) << "In iteration: " << i;
  }
}

TEST(ToolsSuite, reqresion_test1)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0, 0),
        BSpline::Point(1, 1),
        BSpline::Point(2, 2)});

  for (int i = 3; i < 10; ++i)
    b_spline.addPoint(BSpline::Point(i,
                                          i));

  b_spline.erase(5);
  EXPECT_FALSE(b_spline.isInRange(4));
  EXPECT_TRUE(b_spline.isInRange(5));

  b_spline.eraseUpper(4);
  for (int i = 0; i < 10; ++i)
    EXPECT_FALSE(b_spline.isInRange(i)) << i << " is currently exist in range.";
}

TEST(ToolsSuite, reqresion_test2)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0.039682, 2.67),
        BSpline::Point(0.139682, 6.62),
        BSpline::Point(0.239682, 2.55)},
    -M_PI,
    M_PI,
    10.0);

  b_spline.erase(0.098773);
  b_spline.eraseUpper(0.039652);

  b_spline.addPoint(BSpline::Point(0.139652, -1.17));
  b_spline.addPoint(BSpline::Point(0.239652, -1.12));
  b_spline.addPoint(BSpline::Point(0.339652, -1.07));
  b_spline.addPoint(BSpline::Point(0.439652, -1.02));

  EXPECT_GT(b_spline.getValue(0.199121), -1.17);
  EXPECT_LT(b_spline.getValue(0.199121), -1.02);
}

TEST(ToolsSuite, regresion_test3)
{
  using roboland::tools::BSpline;

  BSpline b_spline({BSpline::Point(0.318023, 3.091593),
        BSpline::Point(0.418023, 3.141593),
        BSpline::Point(0.518023, -3.091593)},
    -M_PI,
    M_PI,
    10.0);

  b_spline.addPoint(BSpline::Point(0.618023, -3.041593));
  b_spline.addPoint(BSpline::Point(0.718023, -2.991593));
  b_spline.addPoint(BSpline::Point(0.818023, -2.941593));
  b_spline.addPoint(BSpline::Point(0.918023, -2.891593));
  b_spline.addPoint(BSpline::Point(1.018023, -2.841593));
  b_spline.addPoint(BSpline::Point(1.118023, -2.791593));
  b_spline.addPoint(BSpline::Point(1.218023, -2.741593));

  b_spline.erase(0.434099);

  for (double time = 0.448023; time < 1.218023; time += 0.1)
  {
    EXPECT_GT(b_spline.getValue(time), -M_PI);
    EXPECT_LE(b_spline.getValue(time), M_PI);
  }
}
