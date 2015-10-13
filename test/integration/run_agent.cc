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

using namespace ignition;

class IntegrationTest : public gazebo::ServerFixture
{
  public: void Wait(const int _msec = 500)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(_msec));
  }

  public: void LoadWorld(const std::string &_path)
  {
    this->Load(_path);
    this->world = gazebo::physics::get_world("default");
    EXPECT_TRUE(this->world != NULL);
  }

  public: virtual void SetUp()
  {
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

  public: const std::string testPath = "../test/integration/";

  public: std::shared_ptr<ClientAgent> agent;

  public: std::shared_ptr<ClientAgent> oppAgent;

  public: gazebo::physics::WorldPtr world;
};


/// \brief This tests whether loading the world plugin is successful or not.
TEST_F(IntegrationTest, TestLoadWorldPlugin)
{
  this->LoadWorld(this->testPath + "TestLoadWorldPlugin.world");
  SUCCEED();
}

/// \brief This tests whether two agents can successfully connect, init, beam.
TEST_F(IntegrationTest, TestLoadConnectAgent)
{
  this->LoadWorld(this->testPath + "TestLoadConnectAgent.world");
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

/// \brief This tests whether monitor messages work.
TEST_F(IntegrationTest, TestMonitor)
{
  this->LoadWorld(this->testPath + "TestLoadConnectAgent.world");
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

/// \brief This tests whether we can transition from playOn to kickin
TEST_F(IntegrationTest, TestTransition_PlayOn_KickIn)
{
  this->LoadWorld(this->testPath + "TestLoadWorldPlugin.world");

  this->Wait();
  this->agent->InitAndBeam(0, 0, 0);
  this->agent->ChangePlayMode("PlayOn");
  this->agent->Dribble(math::Vector3d(3, -5, 0.35),
                       math::Vector3d(5, -12, 0.35), 20);
  this->agent->Start();
  this->Wait(2500);

  const auto &lastMsg = this->agent->allMsgs.back();
  EXPECT_NE(lastMsg.find("KickInRight"), std::string::npos);
}

/// \brief Test whether we can transition from kickOff to playOn and then
/// back to kick off due to double touch violation.
TEST_F(IntegrationTest, TestTransition_KickOff_PlayOn)
{
  this->LoadWorld(this->testPath + "TestLoadWorldPlugin.world");

  this->Wait();
  this->agent->InitAndBeam(0, 0, 0);
  this->agent->ChangePlayMode("KickOffLeft");
  this->agent->Dribble(math::Vector3d(0, 0, 0.35),
                       math::Vector3d(0, 1, 0.35), 10);
  this->agent->Start();
  this->Wait(1000);

  bool playOn = false;
  bool kickOffRight = false;
  for (const auto &msg : this->agent->allMsgs)
  {
    if (msg.find("PlayOn"))
    {
      playOn = true;
    }
    if (playOn && msg.find("KickOffRight"))
    {
      kickOffRight = true;
    }
  }
  EXPECT_TRUE(playOn);
  EXPECT_TRUE(kickOffRight);
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
