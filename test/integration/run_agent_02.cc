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

/// \brief This tests whether two agents can successfully connect, init, beam.
TEST_F(IntegrationTest, TestLoadConnectAgent)
{
  this->LoadWorld("TestLoadConnectAgent.world");
  this->Wait();
  this->agent->InitAndBeam(1, 1, 90);
  this->agent->Start();
  this->oppAgent->InitAndBeam(-1, -1, 45);
  this->oppAgent->Start();

  while (this->agent->allMsgs.size() == 0u)
  {
    this->Wait();
  }

  EXPECT_TRUE(this->agent->running);
  EXPECT_TRUE(this->agent->connected);

  EXPECT_GT(this->agent->allMsgs.size(), 0u);
  auto lastMsg = this->agent->allMsgs.back();
  EXPECT_NE(lastMsg.find("GS"), std::string::npos);
  EXPECT_NE(lastMsg.find("BeforeKickOff"), std::string::npos);
  EXPECT_NE(lastMsg.find("myorien"), std::string::npos);
  EXPECT_NE(lastMsg.find("mypos"), std::string::npos);
  EXPECT_NE(lastMsg.find("ballpos"), std::string::npos);

  bool see = false;
  for (const auto &msg : this->agent->allMsgs)
  {
    if (msg.find("See"))
    {
      see = true;
    }
  }

  EXPECT_TRUE(see);

  EXPECT_TRUE(this->oppAgent->running);
  EXPECT_TRUE(this->oppAgent->connected);

  EXPECT_GT(this->oppAgent->allMsgs.size(), 0u);
  lastMsg = this->oppAgent->allMsgs.back();
  EXPECT_NE(lastMsg.find("GS"), std::string::npos);
  EXPECT_NE(lastMsg.find("BeforeKickOff"), std::string::npos);
  EXPECT_NE(lastMsg.find("myorien"), std::string::npos);
  EXPECT_NE(lastMsg.find("mypos"), std::string::npos);
  EXPECT_NE(lastMsg.find("ballpos"), std::string::npos);

  see = false;
  for (const auto &msg : this->oppAgent->allMsgs)
  {
    if (msg.find("See"))
    {
      see = true;
    }
  }
  EXPECT_TRUE(see);
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
