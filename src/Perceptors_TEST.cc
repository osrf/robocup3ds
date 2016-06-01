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
#include <netinet/in.h>
#include <cmath>
#include <memory>
#include <string>
#include <vector>
#include <ignition/math.hh>

#include "gtest/gtest.h"
#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Geometry.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/SoccerField.hh"

using namespace ignition;

/// \brief This test fixture sets up a gameState object and perceptor object
class PerceptorTest : public ::testing::Test
{
  protected: virtual void SetUp()
  {
    GameState::useCounterForGameTime = true;
    this->gameState = std::make_shared<GameState>();
    this->perceptor = std::make_shared<Perceptor>(this->gameState.get());
  }

  protected: virtual bool UpdateLine_Test(Agent &_agent,
                                          const math::Line3<double> &_line,
                                          math::Line3<double> &_testLine)
  {
    this->perceptor->SetG2LMat(_agent);
    _agent.percept.fieldLines.clear();
    this->perceptor->UpdateLine(_agent, _line);
    if (_agent.percept.fieldLines.size() == 0)
    {
      return false;
    }
    _testLine.Set(
      Geometry::PolarToCart(_agent.percept.fieldLines.at(0)[0]),
      Geometry::PolarToCart(_agent.percept.fieldLines.at(0)[1]));
    return true;
  }

  protected: virtual bool UpdateLandmark_Test(Agent &_agent,
                                         const math::Vector3<double> &_landmark,
                                         math::Vector3<double> &_testLandmark)
  {
    this->perceptor->SetG2LMat(_agent);
    _agent.percept.landMarks.clear();
    this->perceptor->UpdateLandmark(_agent, "test", _landmark);
    if (_agent.percept.landMarks.empty())
    {
      return false;
    }
    _testLandmark = Geometry::PolarToCart(_agent.percept.landMarks["test"]);
    return true;
  }

  protected: virtual void TearDown() {}
  protected: std::shared_ptr<GameState> gameState;
  protected: std::shared_ptr<Perceptor> perceptor;
};

/// \brief Test whether Perceptor constructor and destructor works
TEST_F(PerceptorTest, Perceptor_construct_delete)
{
  SUCCEED();
}

/// \brief Test that the view frustum is set correctly based on HFov and VFov
TEST_F(PerceptorTest, Perceptor_SetViewFrustum)
{
  GameState::restrictVision = true;
  GameState::HFov = 0;
  GameState::VFov = 90;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  const std::vector <ignition::math::Plane<double> > &viewFrustum =
    perceptor->GetViewFrustum();

  EXPECT_EQ(viewFrustum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));
  EXPECT_EQ(viewFrustum.at(1).Normal(),
            math::Vector3<double>(0, 1, 0));
  EXPECT_EQ(viewFrustum.at(3).Normal(),
            math::Vector3<double>(0, -1, 0));

  GameState::HFov = 90;
  GameState::VFov = 0;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  EXPECT_EQ(viewFrustum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));
  EXPECT_EQ(viewFrustum.at(2).Normal(),
            math::Vector3<double>(0, 0, -1));
  EXPECT_EQ(viewFrustum.at(4).Normal(),
            math::Vector3<double>(0, 0, 1));

  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  EXPECT_EQ(viewFrustum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));

  EXPECT_GT(viewFrustum.at(1).Normal().Y(), 0.0);
  EXPECT_DOUBLE_EQ(viewFrustum.at(1).Normal().Z(), 0.0);
  EXPECT_GT(viewFrustum.at(1).Normal().X(), 0.0);

  EXPECT_LT(viewFrustum.at(2).Normal().Z(), 0.0);
  EXPECT_DOUBLE_EQ(viewFrustum.at(2).Normal().Y(), 0.0);
  EXPECT_GT(viewFrustum.at(2).Normal().X(), 0.0);

  EXPECT_LT(viewFrustum.at(3).Normal().Y(), 0.0);
  EXPECT_DOUBLE_EQ(viewFrustum.at(3).Normal().Z(), 0.0);
  EXPECT_GT(viewFrustum.at(3).Normal().X(), 0.0);

  EXPECT_GT(viewFrustum.at(4).Normal().Z(), 0.0);
  EXPECT_DOUBLE_EQ(viewFrustum.at(4).Normal().Y(), 0.0);
  EXPECT_GT(viewFrustum.at(4).Normal().X(), 0.0);
}

/// \brief Test whether update line method works when there are no
/// restrictions on vision and that transformation from global coordinates to
/// local coordinates are working correctly
TEST_F(PerceptorTest, Perceptor_UpdateLine_Norestrictvis)
{
  math::Line3<double> origLine, gdLine, testLine;
  GameState::restrictVision = false;
  gameState->AddAgent(1, "blue");
  Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive y-axis
  agent.cameraPos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, M_PI / 2);

  // with noise, we dont expect them to be equal
  Perceptor::useNoise = true;
  // we have line on positive y-axis in global coordinates
  origLine.Set(0, 1, 0, 0, 2, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 2, 0, 0);
  EXPECT_NE(testLine, gdLine);

  // without noies, they are now equal
  Perceptor::useNoise = false;
  origLine.Set(0, 1, 0, 0, 2, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 2, 0, 0);
  EXPECT_EQ(testLine, gdLine);

  // agent's camera is facing down negative x-axis
  agent.cameraRot.Euler(0, 0, M_PI);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(0, -1, 0, 0, -2, 0);
  EXPECT_EQ(testLine, gdLine);

  // line now in front of agent again
  origLine.Set(-1, 0, 0, -2, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 2, 0, 0);
  EXPECT_EQ(testLine, gdLine);

  // line is now parallel with viewing plane
  origLine.Set(-1, 0, 0, -1, 1, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 1, -1, 0);
  EXPECT_EQ(testLine, gdLine);

  // offset agent by 1 in every axis, same with line
  agent.cameraPos.Set(1, 1, 1);
  origLine.Set(0, 1, 1, 0, 2, 1);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 1, -1, 0);
  EXPECT_EQ(testLine, gdLine);
}

/// \brief Test that line clipping works correctly
TEST_F(PerceptorTest, Perceptor_UpdateLine_Restrictvis)
{
  math::Line3<double> origLine, gdLine, testLine;
  GameState::restrictVision = true;
  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  Perceptor::useNoise = false;
  gameState->AddAgent(1, "blue");
  Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive x-axis
  agent.cameraPos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, 0);

  // we have line spanning Y-axis, must be clipped
  origLine.Set(1, -999, 0, 1, 999, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, -1, 0, 1, 1, 0);
  EXPECT_EQ(testLine, gdLine);

  // we have line spanning positive X-axis, does not need to be clipped
  origLine.Set(1, 0, 0, 999, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 999, 0, 0);
  EXPECT_EQ(testLine, gdLine);

  // we have line spanning X-axis, need to be clipped
  origLine.Set(-999, 0, 0, 999, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(0, 0, 0, 999, 0, 0);
  EXPECT_EQ(testLine, gdLine);

  // we have line spanning Z-axis, should be clipped
  origLine.Set(1, 0, -999, 1, 0, 999);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, -1, 1, 0, 1);
  EXPECT_EQ(testLine, gdLine);

  // we have line behind camera, should not be added to fieldLines
  origLine.Set(-1, -999, 0, -1, 999, 0);
  EXPECT_FALSE(UpdateLine_Test(agent, origLine, testLine));

  // we have diagonal line that needs to be clipped on both Z and Y axis
  origLine.Set(1, -999, -999, 1, 999, 999);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, -1, -1, 1, 1, 1);
  EXPECT_EQ(testLine, gdLine);
}

/// \Brief Test whether update landmarks and transformation from global to
/// local works
TEST_F(PerceptorTest, Perceptor_UpdateLandmark_Norestrictvis)
{
  math::Vector3<double> origLandmark, gdLandmark, testLandmark;
  GameState::restrictVision = false;
  Perceptor::useNoise = false;
  gameState->AddAgent(1, "blue");
  Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive y-axis
  agent.cameraPos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, M_PI / 2);

  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(0, -1, 0);
  EXPECT_EQ(gdLandmark, testLandmark);

  origLandmark.Set(1, 1, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(1, -1, 0);
  EXPECT_EQ(gdLandmark, testLandmark);

  agent.cameraRot.Euler(0, 0, M_PI / 4);
  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(1 / sqrt(2), -1 / sqrt(2), 0);
  EXPECT_EQ(gdLandmark, testLandmark);

  agent.cameraPos.Set(1, 2, 3);
  origLandmark.Set(1, 0, 0);
  agent.cameraRot.Euler(0, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(0, -2, -3);
}

/// \Brief Test whether update landmark clipping works correctly
TEST_F(PerceptorTest, Perceptor_UpdateLandmark_Restrictvis)
{
  math::Vector3<double> origLandmark, gdLandmark, testLandmark;
  GameState::restrictVision = true;
  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  Perceptor::useNoise = false;
  gameState->AddAgent(1, "blue");
  Agent &agent = gameState->teams.at(0)->members.at(0);

  agent.cameraPos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, 0);

  origLandmark.Set(-999, 0, 0);
  EXPECT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(-1, 0, 0);
  EXPECT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(0, 999, 0);
  EXPECT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(0, 0, -999);
  EXPECT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(2, 1, 1);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark = origLandmark;
  EXPECT_EQ(gdLandmark, testLandmark);

  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark = origLandmark;
  EXPECT_EQ(gdLandmark, testLandmark);
}

/// \Brief Test whether updating position of other agent body parts works as
/// intended
TEST_F(PerceptorTest, Percepter_UpdateOtherAgent)
{
  GameState::HFov = 89;
  GameState::VFov = 89;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);
  Perceptor::useNoise = false;
  gameState->AddAgent(1, "blue");
  gameState->AddAgent(1, "red");
  Agent &agent1 = gameState->teams.at(0)->members.at(0);
  Agent &agent2 = gameState->teams.at(1)->members.at(0);
  agent1.cameraPos.Set(0, 0, 0);
  agent1.cameraRot.Euler(0, 0, M_PI / 2);
  perceptor->SetG2LMat(agent1);
  agent2.cameraPos.Set(1, 1, 0);
  agent2.selfBodyMap["HEAD"] = agent2.cameraPos +
                               math::Vector3<double>(0, 0, 1);
  agent2.selfBodyMap["BODY"] = agent2.cameraPos;
  AgentId agent2Id(1, "red");

  // without restricted vision, other agent's body parts should be visible
  GameState::restrictVision = false;
  perceptor->UpdateOtherAgent(agent1, agent2);
  EXPECT_NE(agent1.percept.otherAgentBodyMap.find(agent2Id),
            agent1.percept.otherAgentBodyMap.end());
  EXPECT_NE(agent1.percept.otherAgentBodyMap[agent2Id].find("HEAD"),
            agent1.percept.otherAgentBodyMap[agent2Id].end());
  EXPECT_NE(agent1.percept.otherAgentBodyMap[agent2Id].find("BODY"),
            agent1.percept.otherAgentBodyMap[agent2Id].end());
  EXPECT_EQ(
    Geometry::PolarToCart(agent1.percept.otherAgentBodyMap[agent2Id]["HEAD"]),
    math::Vector3<double>(1, -1, 1));
  EXPECT_EQ(
    Geometry::PolarToCart(agent1.percept.otherAgentBodyMap[agent2Id]["BODY"]),
    math::Vector3<double>(1, -1, 0));

  // with restricted vision, other agent's body parts should not be visible
  GameState::restrictVision = true;
  agent1.percept.otherAgentBodyMap.clear();
  perceptor->UpdateOtherAgent(agent1, agent2);
  EXPECT_EQ(agent1.percept.otherAgentBodyMap.find(agent2Id),
            agent1.percept.otherAgentBodyMap.end());
}

/// \Brief Test whether updating position of messages heard from other agents
/// is working as intended
TEST_F(PerceptorTest, Percepter_UpdateAgentHear)
{
  gameState->AddAgent(1, "red");
  gameState->AddAgent(2, "red");
  gameState->SetCycleCounter(0);
  const auto &team = gameState->teams.at(0);
  Agent &agent1 = team->members.at(0);
  Agent &agent2 = team->members.at(1);
  agent1.cameraPos.Set(5, 0, 0);
  agent1.cameraRot.Euler(0, 0, M_PI / 2);
  agent2.cameraPos.Set(0, 100, 0);
  team->say.agentId = std::make_pair(5, "red");
  team->say.pos.Set(0, 0, 0);
  team->say.msg = "hello";
  team->say.isValid = true;

  perceptor->SetG2LMat(agent1);
  perceptor->UpdateAgentHear(agent1);
  EXPECT_TRUE(agent1.percept.hear.isValid);
  EXPECT_DOUBLE_EQ(agent1.percept.hear.gameTime,
                   gameState->GetElapsedGameTime());
  EXPECT_DOUBLE_EQ(agent1.percept.hear.yaw, IGN_DTOR(90.0));
  EXPECT_FALSE(agent1.percept.hear.self);
  EXPECT_EQ(agent1.percept.hear.msg, "hello");

  team->say.agentId = std::make_pair(1, "red");
  perceptor->UpdateAgentHear(agent1);
  EXPECT_TRUE(agent1.percept.hear.self);

  team->say.isValid = false;
  perceptor->UpdateAgentHear(agent1);
  EXPECT_FALSE(agent1.percept.hear.isValid);

  perceptor->SetG2LMat(agent2);
  team->say.isValid = true;
  perceptor->UpdateAgentHear(agent2);
  EXPECT_FALSE(agent2.percept.hear.isValid);

  team->say.isValid = false;
  perceptor->UpdateAgentHear(agent2);
  EXPECT_FALSE(agent2.percept.hear.isValid);
}

/// \Brief Test whether the Update() function works
TEST_F(PerceptorTest, Percepter_Update)
{
  std::string teamNames[2] = {"blue", "red"};
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      gameState->AddAgent(j + 1, teamNames[i]);
      auto &agent = gameState->teams.at(i)->members.at(j);
      agent.selfBodyMap["BODY"] = agent.pos;
    }
  }
  gameState->SetCycleCounter(0);
  const auto &team = gameState->teams.at(0);
  team->say.agentId = std::make_pair(1, "blue");
  team->say.pos.Set(0, 0, 0);
  team->say.msg = "hello";
  team->say.isValid = true;

  GameState::restrictVision = false;
  Perceptor::useNoise = false;
  perceptor->Update();
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      auto &agent = gameState->teams.at(i)->members.at(j);
      EXPECT_EQ(agent.percept.fieldLines.size(),
                SoccerField::kFieldLines.size());
      EXPECT_EQ(agent.percept.landMarks.size(),
                SoccerField::kLandMarks.size() + 1u);
      EXPECT_NE(agent.percept.landMarks.find("B"),
                agent.percept.landMarks.end());
      EXPECT_EQ(agent.percept.otherAgentBodyMap.size(), 21u);
      EXPECT_TRUE(agent.percept.hear.isValid);
      EXPECT_DOUBLE_EQ(agent.percept.hear.gameTime,
                       gameState->GetElapsedGameTime());
      EXPECT_DOUBLE_EQ(agent.percept.hear.yaw, 0.0);
      EXPECT_EQ(agent.percept.hear.msg, "hello");
      if (j == 0 && i == 0)
      {
        EXPECT_TRUE(agent.percept.hear.self);
      }
      else
      {
        EXPECT_FALSE(agent.percept.hear.self);
      }
    }
  }
}

/// \Brief Test whether the Serialize() function works
TEST_F(PerceptorTest, Percepter_Serialize)
{
  std::string teamNames[2] = {"red", "blue"};
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      gameState->AddAgent(j + 1, teamNames[i]);
      auto &agent = gameState->teams.at(i)->members.at(j);
      if (i == 0 && j == 0)
      {
        continue;
      }
      gameState->MoveAgent(agent, math::Vector3<double>(5.0, 1.0,
                           0.35));
      agent.selfBodyMap["BODY"] = agent.pos;
      agent.selfBodyMap["HEAD"] = agent.pos +
                                  math::Vector3<double>(0, 0, 1);
    }
  }
  GameState::groundTruthInfo = true;
  GameState::restrictVision = true;
  gameState->SetCycleCounter(1);
  const auto &team = gameState->teams.at(1);
  team->say.agentId = std::make_pair(1, "blue");
  team->say.pos.Set(5.0, 1.0, 0.35);
  team->say.msg = "hello";
  team->say.isValid = true;
  Perceptor::useNoise = false;
  Perceptor::updateVisualFreq = 1;
  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustum(GameState::HFov, GameState::VFov);

  auto &redAgent = gameState->teams.at(0)->members.at(0);
  redAgent.percept.hingeJoints["joint1"] = 0.5;
  redAgent.percept.hingeJoints["joint2"] = 0.2;
  perceptor->SetG2LMat(redAgent);
  perceptor->Update();

  char testString[16384];
  int cx;

  // currently takes around 1.2 milliseconds to finish for all 22 agents
  for (int i = 0; i < 1000; ++i)
  {
    cx = perceptor->Serialize(redAgent, testString, sizeof(testString));
  }
  std::cout << testString << std::endl;

  // check that certain names are there in string
  EXPECT_TRUE(strstr(testString, "joint1"));
  EXPECT_TRUE(strstr(testString, "joint2"));
  EXPECT_TRUE(strstr(testString, "HEAD"));
  EXPECT_TRUE(strstr(testString, "BODY"));
  EXPECT_TRUE(strstr(testString, "hear"));
  EXPECT_TRUE(strstr(testString, "myorien"));
  EXPECT_TRUE(strstr(testString, "ballpos"));
  EXPECT_TRUE(strstr(testString, "mypos"));

  // check that parenthesis are balanced
  int paren = 0;
  for (size_t i = 0; i < sizeof(testString); ++i)
  {
    if (testString[i] == '\0')
    {
      break;
    }
    if (testString[i] == '(')
    {
      paren++;
    }
    else if (testString[i] == ')')
    {
      paren--;
    }
  }
  EXPECT_EQ(0, paren);

  // make sure returned length is correct
  EXPECT_EQ(static_cast<int>(strlen(testString)), cx);

  // same test for other team
  auto &blueAgent = gameState->teams.at(1)->members.at(0);
  blueAgent.percept.hingeJoints["joint1"] = 0.5;
  blueAgent.percept.hingeJoints["joint2"] = 0.2;
  cx = perceptor->Serialize(blueAgent, testString, sizeof(testString));

  // check that certain names are there in string
  EXPECT_TRUE(strstr(testString, "joint1"));
  EXPECT_TRUE(strstr(testString, "joint2"));
  EXPECT_TRUE(strstr(testString, "HEAD"));
  EXPECT_TRUE(strstr(testString, "BODY"));
  EXPECT_TRUE(strstr(testString, "hear"));
  EXPECT_TRUE(strstr(testString, "self"));
  EXPECT_TRUE(strstr(testString, "myorien"));
  EXPECT_TRUE(strstr(testString, "ballpos"));
  EXPECT_TRUE(strstr(testString, "mypos"));

  // ensure that encoding is correct
  char b[400];
  unsigned int _cx = htonl(static_cast<unsigned int>(cx));
  memcpy(b, &_cx, 4);
  unsigned int _cx2;
  memcpy(&_cx2, b, 4);
  int cx2 = static_cast<int>(ntohl(_cx2));
  EXPECT_EQ(cx, cx2);
}

/// \brief Test accelerometer
TEST_F(PerceptorTest, Perceptor_Accelerometer)
{
  this->gameState->AddAgent(1, "blue");
  auto agent = gameState->teams.at(0)->members.at(0);

  // Start with zero acceleration
  EXPECT_EQ(agent.percept.accel, ignition::math::Vector3d::Zero);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
