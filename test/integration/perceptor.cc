/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <memory>
#include <gazebo/physics/physics.hh>
#include <gazebo/test/ServerFixture.hh>

#include "robocup3ds/ClientAgent.hh"
#include "test/test_config.h"

class PerceptorTest : public gazebo::ServerFixture
{
  //////////////////////////////////////////////////
  public: void Wait(const int _msec = 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(_msec));
  }

  //////////////////////////////////////////////////
  public: void LoadWorld(const std::string &_path)
  {
    this->Load(_path, true);
    this->world = gazebo::physics::get_world("default");
    EXPECT_TRUE(this->world != NULL);
    this->world->Step(1);
    this->world->SetPaused(false);
  }

  //////////////////////////////////////////////////
  public: virtual void SetUp()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      ROBOCUP3DS_TEST_INTEGRATION_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      ROBOCUP3DS_TEST_WORLD_PATH);
    gazebo::common::SystemPaths::Instance()->AddModelPaths(
      ROBOCUP3DS_TEST_MODEL_PATH);
  }

  //////////////////////////////////////////////////
  public: virtual void TearDown()
  {
    this->agent.reset();
    this->world.reset();
    gazebo::ServerFixture::TearDown();
    this->Wait();
  }

  public: std::unique_ptr<ClientAgent> agent;

  public: gazebo::physics::WorldPtr world;
};

/////////////////////////////////////////////////
/// \brief .
TEST_F(PerceptorTest, IMU)
{
  this->LoadWorld("TestPerceptor.world");

  this->agent.reset(new ClientAgent(
                    "0.0.0.0", 3100, 3200, 1, "red", "left"));
  this->Wait();
  this->agent->InitAndBeam(1, 1, 90);
  this->agent->Start();

  while (this->agent->allMsgs.size() == 0u)
    this->Wait();

  EXPECT_TRUE(this->agent->running);
  EXPECT_TRUE(this->agent->connected);

  gazebo::physics::WorldPtr world = gazebo::physics::get_world("default");
  ASSERT_TRUE(world != NULL);

  this->Wait(1500);

  this->agent->ChangePlayMode("PlayOn");

  this->Wait(1000);
}

/////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
