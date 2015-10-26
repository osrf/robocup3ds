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

/// \brief This tests whether monitor messages work.
TEST_F(IntegrationTest, TestMonitor)
{
  this->LoadWorld("TestLoadConnectAgent.world");
  this->Wait();
  this->agent->InitAndBeam(1, 1, 90);
  this->agent->ChangePlayMode("PlayOn");
  this->agent->Start();

  // test game mode has successfully changed
  while (this->agent->allMsgs.size() == 0u)
  {
    this->Wait();
  }

  const auto &lastMsg = this->agent->allMsgs.back();
  EXPECT_NE(lastMsg.find("PlayOn"), std::string::npos);

  // test that MoveBall and MoveAgent work.
  this->agent->MoveBall(math::Vector3d(1.35, 5.69, 0.042));
  this->agent->MoveAgent(math::Vector3d(-7.35, -11.69, 0.35));
  this->Wait();

  bool gd = false;
  for (const auto &msg : this->agent->allMsgs)
  {
    if (msg.find("1.35 5.69 0.04") && msg.find("-7.35 -11.69 0.35"))
    {
      gd = true;
    }
  }
  EXPECT_TRUE(gd);

  size_t numMessages;
  {
    std::lock_guard<std::mutex> lock(this->agent->mutex);
    numMessages = this->agent->allMsgs.size();
  }
  this->agent->RemoveAgent();
  size_t currMsgCount = numMessages;
  while (this->agent->allMsgs.size() != currMsgCount ||
         this->agent->allMsgs.size() == numMessages)
  {
    this->Wait();
    currMsgCount = this->agent->allMsgs.size();
  }
  EXPECT_LE(currMsgCount - numMessages, 3u);
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
