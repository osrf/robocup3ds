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
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <gtest/gtest.h>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <map>
#include <string>

#include "robocup3ds/Util.hh"

using namespace Util;

/// \brief We test that the functions that convert between gazebo and ignition
/// math objects work
TEST(UtilTest, I2G_G2I_Test)
{
  auto pt1 = gazebo::math::Vector3(5, 6, 8);
  auto pt2 = G2I(pt1);
  EXPECT_DOUBLE_EQ(pt1[0], pt2.X());
  EXPECT_DOUBLE_EQ(pt1[1], pt2.Y());
  EXPECT_DOUBLE_EQ(pt1[2], pt2.Z());
  auto pt3 = I2G(pt2);
  EXPECT_EQ(pt1, pt3);

  auto q1 = gazebo::math::Quaternion(5, 6, 8);
  auto q2 = G2I(q1);
  EXPECT_DOUBLE_EQ(q1.w, q2.W());
  EXPECT_DOUBLE_EQ(q1.x, q2.X());
  EXPECT_DOUBLE_EQ(q1.y, q2.Y());
  EXPECT_DOUBLE_EQ(q1.z, q2.Z());
  auto q3 = I2G(q2);
  EXPECT_EQ(q1, q3);

  auto p1 = gazebo::math::Pose(6, 7, 3, 5, 6, 8);
  auto p2 = G2I(p1);
  EXPECT_DOUBLE_EQ(p1.rot.w, p2.Rot().W());
  EXPECT_DOUBLE_EQ(p1.rot.x, p2.Rot().X());
  EXPECT_DOUBLE_EQ(p1.rot.y, p2.Rot().Y());
  EXPECT_DOUBLE_EQ(p1.rot.z, p2.Rot().Z());
  EXPECT_DOUBLE_EQ(p1.pos.x, p2.Pos().X());
  EXPECT_DOUBLE_EQ(p1.pos.y, p2.Pos().Y());
  EXPECT_DOUBLE_EQ(p1.pos.z, p2.Pos().Z());
  auto p3 = I2G(p2);
  EXPECT_EQ(p1, p3);
}

/// \brief We test that the functions that extract values from a map work
TEST(UtilTest, MapValueTest)
{
  double v;
  bool b;
  std::map<std::string, std::string> testMap;
  testMap["key1"] = "567.3";
  EXPECT_TRUE(LoadConfigParameter(testMap, "key1", v));
  EXPECT_DOUBLE_EQ(v, 567.3);

  testMap["key1"] = "asdlfas99009";
  EXPECT_FALSE(LoadConfigParameter(testMap, "key1", v));

  testMap["key1"] = "63.390 35.46";
  EXPECT_FALSE(LoadConfigParameter(testMap, "key1", v));

  testMap["key2"] = "false";
  EXPECT_TRUE(LoadConfigParameterBool(testMap, "key2", b));
  EXPECT_FALSE(b);

  testMap["key2"] = "1";
  EXPECT_TRUE(LoadConfigParameterBool(testMap, "key2", b));
  EXPECT_TRUE(b);

  testMap["key2"] = "089fas09";
  EXPECT_FALSE(LoadConfigParameterBool(testMap, "key2", b));
}

/// \brief We test that the function to convert c-string to double works
TEST(UtilTest, Str2DTest)
{
  double v;
  char x[50] = "6.53993";
  EXPECT_TRUE(S2D(x, v));
  EXPECT_DOUBLE_EQ(v, 6.53993);

  char x2[50] = "5.2393 239420.0";
  EXPECT_FALSE(S2D(x2, v));

  char x3[50] = "2w0r924252";
  EXPECT_FALSE(S2D(x3, v));
}

int main(int argc, char **argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
