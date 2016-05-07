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
#include <iostream>
#include <memory>
#include <string>
#include <thread>
#include <gazebo/physics/World.hh>
#include <gazebo/test/ServerFixture.hh>
#include <ignition/math.hh>
#include "robocup3ds/ClientAgent.hh"
#include "test/test_config.h"

using namespace ignition;

class IntegrationTest : public gazebo::ServerFixture
{
  public: void Wait(const int _msec = 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(_msec));
  }

  public: void LoadWorld(const std::string &_path)
  {
    this->Load(_path, true);
    this->world = gazebo::physics::get_world("default");
    EXPECT_TRUE(this->world != NULL);
    this->world->Step(1);
    this->world->SetPaused(false);
  }

  public: virtual void SetUp()
  {
    gazebo::common::SystemPaths::Instance()->AddPluginPaths(
      ROBOCUP3DS_TEST_INTEGRATION_PATH);
    gazebo::common::SystemPaths::Instance()->AddGazeboPaths(
      ROBOCUP3DS_TEST_WORLD_PATH);
    gazebo::common::SystemPaths::Instance()->AddModelPaths(
      ROBOCUP3DS_TEST_MODEL_PATH);

    this->agent = std::make_shared<ClientAgent>(
                    "0.0.0.0", 3100, 3200, 1, "red", "left");
    this->oppAgent = std::make_shared<ClientAgent>(
                       "0.0.0.0", 3100, 3200, 1, "blue", "right");
    this->Wait();
  }

  public: virtual void TearDown()
  {
    this->agent.reset();
    this->oppAgent.reset();
    this->world.reset();
    gazebo::ServerFixture::TearDown();
    this->Wait();
  }

  public: std::shared_ptr<ClientAgent> agent;

  public: std::shared_ptr<ClientAgent> oppAgent;

  public: gazebo::physics::WorldPtr world;
};

/// \brief This tests whether we can transition from playOn to kickin
TEST_F(IntegrationTest, TestTransition_PlayOn_KickIn)
{
  this->LoadWorld("TestLoadWorldPlugin.world");

  this->Wait();
  this->agent->InitAndBeam(0, 0, 0);
  this->agent->ChangePlayMode("PlayOn");
  this->agent->Dribble(math::Vector3d(3, -5, 0.35),
                       math::Vector3d(5, -12, 0.35), 20);
  this->agent->Start();
  this->Wait(2500);

  const auto &lastMsg = this->agent->allMsgs.back();
  EXPECT_NE(lastMsg.find("KickIn_Right"), std::string::npos);
}

//////////////////////////////////////////////////
int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ignition::math::Rand::Seed(42);
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
