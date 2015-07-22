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
#include <gazebo/physics/World.hh>
#include <gazebo/test/ServerFixture.hh>
#include <ignition/math.hh>
#include <string>

using namespace ignition;
using namespace std;

class IntegrationTest : public gazebo::ServerFixture
{
};

/// \brief This tests whether loading the world plugin is successful or not
TEST_F(IntegrationTest, TestLoadWorldPlugin)
{
  const string worldPath = "/home/jliang/Desktop/OSRF/gazebo_install/share/"
                      "gazebo-6.0/worlds/robocup3d.world";

  this->Load(worldPath);
  const auto &world = gazebo::physics::get_world("default");
  EXPECT_TRUE(world != NULL);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
