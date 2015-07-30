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
#include <chrono>
#include <gazebo/physics/World.hh>
#include <gazebo/test/ServerFixture.hh>
#include <ignition/math.hh>
#include <memory>
#include <string>
#include <thread>

#include "ClientAgent.hh"

using namespace ignition;
using namespace std;

// class IntegrationTest_Basic : public gazebo::ServerFixture
// {
//   public:
//     const string worldPath =
//       "./worlds/robocup3d.world";
// };

class IntegrationTest_Immed : public gazebo::ServerFixture
{
  public:
    virtual void SetUp()
    {
      this->Load(worldPath);
      this->world = gazebo::physics::get_world("default");
      this->agent = make_shared<ClientAgent>("0.0.0.0", 3100, 3200);
    }

  public:
    virtual void TearDown()
    {
      this->agent.reset();
      gazebo::ServerFixture::TearDown();
    }

  public:
    const string worldPath =
      "./worlds/robocup3d.world";

  public:
    shared_ptr<ClientAgent> agent;

  public:
    gazebo::physics::WorldPtr world;
};


/// \brief This tests whether loading the world plugin is successful or not
// TEST_F(IntegrationTest_Basic, TestLoadWorldPlugin)
// {
//   this->Load(worldPath);
//   const auto &world = gazebo::physics::get_world("default");
//   EXPECT_TRUE(world != NULL);
// }

/// \brief This tests whether agent can successfully connect, init, and do some
/// beaming
TEST_F(IntegrationTest_Immed, TestLoadConnectAgent)
{
  const auto &world = gazebo::physics::get_world("default");
  EXPECT_TRUE(world != NULL);

  std::this_thread::sleep_for(std::chrono::milliseconds(300));
  this->agent->Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(300000));

  EXPECT_TRUE(this->agent->running);
  EXPECT_TRUE(this->agent->connected);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
