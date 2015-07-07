/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES or CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions &&
 * limitations under the License.
 *
*/
#include <cmath>
#include <iostream>
#include "ignition/math.hh"

#include "gtest/gtest.h"
#include "robocup3ds/Geometry.hh"

using namespace ignition;
using namespace std;

/// \brief Test whether IntersectionCircunferenceLine function works
TEST(Geometry_Test, IntersectionCircunferenceLine)
{
  // y = 0
  math::Line3<double> line(-999, 0, 999, 0);
  math::Vector3<double> center(0, 0, 0);
  double radius = 1.0;

  math::Vector3<double> pt1, pt2;
  EXPECT_TRUE(
    Geometry::IntersectionCircunferenceLine(line, center, radius, pt1, pt2));
  EXPECT_EQ(pt1, math::Vector3<double>(-1, 0, 0));
  EXPECT_EQ(pt2, math::Vector3<double>(1, 0, 0));

  radius = 2.0;
  EXPECT_TRUE(
    Geometry::IntersectionCircunferenceLine(line, center, radius, pt1, pt2));
  EXPECT_EQ(pt1, math::Vector3<double>(-2, 0, 0));
  EXPECT_EQ(pt2, math::Vector3<double>(2, 0, 0));

  // y = x
  line.Set(-999, -999, 999, 999);
  radius = 1.0;
  EXPECT_TRUE(
    Geometry::IntersectionCircunferenceLine(line, center, radius, pt1, pt2));
  EXPECT_EQ(pt1, math::Vector3<double>(-sin(RAD(45)), -cos(RAD(45)), 0));
  EXPECT_EQ(pt2, math::Vector3<double>(sin(RAD(45)), cos(RAD(45)), 0));

  // x = 5
  line.Set(5, 999, 5, -999);
  center.Set(5, 2, 0);
  EXPECT_TRUE(
    Geometry::IntersectionCircunferenceLine(line, center, radius, pt1, pt2));
  EXPECT_EQ(pt1, math::Vector3<double>(5, 3, 0));
  EXPECT_EQ(pt2, math::Vector3<double>(5, 1, 0));

  // x = 0 does not intersect
  line.Set(0, 999, 0, -999);
  EXPECT_FALSE(
    Geometry::IntersectionCircunferenceLine(line, center, radius, pt1, pt2));
}


/// \brief Test whether PointAbovePlane function works
TEST(Geometry_Test, PointAbovePlane)
{
  math::Plane<double> p(math::Vector3<double>(0, 0, 1));
  EXPECT_TRUE(Geometry::PointAbovePlane(math::Vector3<double>(-1, 1, 1), p));
  EXPECT_FALSE(Geometry::PointAbovePlane(math::Vector3<double>(1, 1, -1), p));

  p.Normal() = math::Vector3<double>(0, -1, 0);
  EXPECT_TRUE(Geometry::PointAbovePlane(math::Vector3<double>(-1, -1, 1), p));
  EXPECT_FALSE(Geometry::PointAbovePlane(math::Vector3<double>(1, 1, -1), p));

  p.Normal() = math::Vector3<double>(1, 0, 0);
  EXPECT_TRUE(Geometry::PointAbovePlane(math::Vector3<double>(1, 1, -1), p));
  EXPECT_FALSE(Geometry::PointAbovePlane(math::Vector3<double>(-1, -1, 1), p));
}

/// \brief Test whether IntersectionPlaneLine function works
TEST(Geometry_Test, IntersectionPlaneLine)
{
  double t;
  math::Vector3<double> pt;

  // simple case
  math::Plane<double> plane(math::Vector3<double>(0, 0, 1));
  math::Line3<double> line(0, 0, -999, 0, 0, 999);
  EXPECT_TRUE(Geometry::IntersectionPlaneLine(line, plane, t, pt));
  EXPECT_DOUBLE_EQ(t, 0.5);
  EXPECT_EQ(pt, math::Vector3<double>::Zero);

  // more complex case
  plane.Normal() = math::Vector3<double>(5, 3, 4);
  line.Set(1, 2, 3, 5, 7, 9);
  EXPECT_TRUE(Geometry::IntersectionPlaneLine(line, plane, t, pt));
  EXPECT_EQ(pt, math::Vector3<double>(-0.5593220338983051,
                                      0.05084745762711851,
                                      0.6610169491525424));

  // another complex case
  plane.Set(math::Vector3<double>(1.5, -1, 2.2),
            math::Vector2<double>(), -2);
  line.Set(1, 2, 3, -0.5, 5, 3.35);
  EXPECT_TRUE(Geometry::IntersectionPlaneLine(line, plane, t, pt));
  EXPECT_EQ(pt, math::Vector3<double>(-0.3727678571428572,
                                      4.745535714285714,
                                      3.3203125));

  // line is parallel with plane
  plane.Set(math::Vector3<double>(1, 0, 0),
            math::Vector2<double>(), 0);
  line.Set(2, -99, 5, 2, 5, 55);
  EXPECT_FALSE(Geometry::IntersectionPlaneLine(line, plane, t, pt));
}

/// \brief Test whether ClipPlaneLine function works
TEST(Geometry_Test, ClipPlaneLine)
{
  math::Line3<double> line(0, 0, -999, 0, 0, 999);
  math::Plane<double> plane(math::Vector3<double>(0, 0, 1));

  EXPECT_TRUE(Geometry::ClipPlaneLine(line, plane));
  ASSERT_EQ(line, math::Line3<double>(0, 0, 0, 0, 0, 999));

  line.Set(0, 0, 1, 0, 0, 2);
  EXPECT_TRUE(Geometry::ClipPlaneLine(line, plane));
  ASSERT_EQ(line, math::Line3<double>(0, 0, 1, 0, 0, 2));

  line.Set(0, 0, 1, 0, 0, -2);
  EXPECT_TRUE(Geometry::ClipPlaneLine(line, plane));
  ASSERT_EQ(line, math::Line3<double>(0, 0, 1, 0, 0, 0));

  line.Set(0, 0, -1, 0, 0, -2);
  EXPECT_FALSE(Geometry::ClipPlaneLine(line, plane));
}

/// \brief Test whether CartToPolar and PolarToCart functions works
TEST(Geometry_Test, CartToPolar_PolarToCart)
{
  math::Vector3<double> pt(5, 0, 0);
  math::Vector3<double> pt2(5, 0, RAD(90));
  ASSERT_EQ(Geometry::CartToPolar(pt), pt2);
  ASSERT_EQ(pt, Geometry::PolarToCart(pt2));

  pt.Set(6, 0, 4);
  pt2.Set(7.211102550928, 0, RAD(56.30993247402));
  ASSERT_EQ(Geometry::CartToPolar(pt), pt2);
  ASSERT_EQ(pt, Geometry::PolarToCart(pt2));

  pt.Set(3, 4, 5);
  pt2.Set(7.0710678118655, RAD(53.130102354156), RAD(45));
  ASSERT_EQ(Geometry::CartToPolar(pt), pt2);
  ASSERT_EQ(pt, Geometry::PolarToCart(pt2));

  pt.Set(-1, 3, 2);
  pt2.Set(3.7416573867739, RAD(108.43494882292), RAD(57.688466762576));
  ASSERT_EQ(Geometry::CartToPolar(pt), pt2);
  ASSERT_EQ(pt, Geometry::PolarToCart(pt2));
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
