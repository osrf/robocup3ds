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
