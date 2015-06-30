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

#include <cmath>
#include <ignition/math.hh>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Geometry.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/SoccerField.hh"

using namespace ignition;
using namespace std;

/// \class PerceptorTest_basic
/// \brief This test fixture sets up a gameState object and perceptor object
class PerceptorTest : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      this->gameState = new GameState();
      this->perceptor = new Perceptor(gameState);
    }

  protected:
    virtual bool UpdateLine_Test(GameState::Agent &_agent,
                                 const math::Line3<double> &_line,
                                 math::Line3<double> &_testLine) const
    {
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

    virtual bool UpdateLandmark_Test(GameState::Agent &_agent,
                                     const math::Vector3<double> &_landmark,
                                     math::Vector3<double> &_testLandmark) const
    {
      _agent.percept.landMarks.clear();
      this->perceptor->UpdateLandmark(_agent, "test", _landmark);
      if (_agent.percept.landMarks.size() == 0)
      {
        return false;
      }
      _testLandmark = Geometry::PolarToCart(_agent.percept.landMarks["test"]);
      return true;
    }

  protected:
    virtual void TearDown()
    {
      delete this->gameState;
      delete this->perceptor;
    }

  protected:
    GameState *gameState;
  protected:
    Perceptor *perceptor;
};

/// \brief Test whether Perceptor constructor && destructor works
TEST_F(PerceptorTest, Perceptor_construct_delete)
{
  SUCCEED();
}

/// \brief Test whether update line method works when there are no
/// restrictions on vision and that transformation from global coordinates to
/// local coordinates are working correctly
TEST_F(PerceptorTest, Perceptor_UpdateLine_Norestrictvis_Transform)
{
  math::Line3<double> origLine, gdLine, testLine;
  GameState::restrictVision = false;
  perceptor->useNoise = false;
  gameState->AddAgent(1, "blue");
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive y-axis
  agent.pos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, M_PI / 2);
  perceptor->SetG2LMat(agent);

  // we have line on positive y-axis in global coordinates
  origLine.Set(0, 1, 0, 0, 2, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 2, 0, 0);
  ASSERT_EQ(testLine, gdLine);

  // agent's camera is facing down negative x-axis
  agent.cameraRot.Euler(0, 0, M_PI);
  perceptor->SetG2LMat(agent);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(0, -1, 0, 0, -2, 0);
  ASSERT_EQ(testLine, gdLine);

  // line now in front of agent again
  origLine.Set(-1, 0, 0, -2, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 2, 0, 0);
  ASSERT_EQ(testLine, gdLine);

  // line is now parallel with viewing plane
  origLine.Set(-1, 0, 0, -1, 1, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 1, -1, 0);
  ASSERT_EQ(testLine, gdLine);

  // offset agent by 1 in every axis, same with line
  agent.pos.Set(1, 1, 1);
  perceptor->SetG2LMat(agent);
  origLine.Set(0, 1, 1, 0, 2, 1);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 1, -1, 0);
  ASSERT_EQ(testLine, gdLine);
}

/// \brief Test that the view frustum is set correctly based on HFov and VFov
TEST_F(PerceptorTest, Perceptor_SetViewFrustrum)
{
  GameState::restrictVision = true;
  GameState::HFov = 0;
  GameState::VFov = 90;
  perceptor->SetViewFrustrum();
  std::vector <ignition::math::Plane<double> > &viewFrustrum =
    perceptor->GetViewFrustrum();

  ASSERT_EQ(viewFrustrum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));
  ASSERT_EQ(viewFrustrum.at(1).Normal(),
            math::Vector3<double>(0, 1, 0));
  ASSERT_EQ(viewFrustrum.at(3).Normal(),
            math::Vector3<double>(0, -1, 0));

  GameState::HFov = 90;
  GameState::VFov = 0;
  perceptor->SetViewFrustrum();
  ASSERT_EQ(viewFrustrum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));
  ASSERT_EQ(viewFrustrum.at(2).Normal(),
            math::Vector3<double>(0, 0, -1));
  ASSERT_EQ(viewFrustrum.at(4).Normal(),
            math::Vector3<double>(0, 0, 1));

  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustrum();
  ASSERT_EQ(viewFrustrum.at(0).Normal(),
            math::Vector3<double>(1, 0, 0));

  ASSERT_GT(viewFrustrum.at(1).Normal().Y(), 0.0);
  ASSERT_DOUBLE_EQ(viewFrustrum.at(1).Normal().Z(), 0.0);
  ASSERT_GT(viewFrustrum.at(1).Normal().X(), 0.0);

  ASSERT_LT(viewFrustrum.at(2).Normal().Z(), 0.0);
  ASSERT_DOUBLE_EQ(viewFrustrum.at(2).Normal().Y(), 0.0);
  ASSERT_GT(viewFrustrum.at(2).Normal().X(), 0.0);

  ASSERT_LT(viewFrustrum.at(3).Normal().Y(), 0.0);
  ASSERT_DOUBLE_EQ(viewFrustrum.at(3).Normal().Z(), 0.0);
  ASSERT_GT(viewFrustrum.at(3).Normal().X(), 0.0);

  ASSERT_GT(viewFrustrum.at(4).Normal().Z(), 0.0);
  ASSERT_DOUBLE_EQ(viewFrustrum.at(4).Normal().Y(), 0.0);
  ASSERT_GT(viewFrustrum.at(4).Normal().X(), 0.0);
}

/// \brief Test that line clipping works correctly
TEST_F(PerceptorTest, Perceptor_UpdateLine_Restrictvis)
{
  math::Line3<double> origLine, gdLine, testLine;
  GameState::restrictVision = true;
  GameState::HFov = 90;
  GameState::VFov = 90;
  perceptor->SetViewFrustrum();
  perceptor->useNoise = false;
  gameState->AddAgent(1, "blue");
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive x-axis
  agent.pos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, 0);
  perceptor->SetG2LMat(agent);

  // we have line spanning Y-axis, must be clipped
  origLine.Set(1, -999, 0, 1, 999, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, -1, 0, 1, 1, 0);
  ASSERT_EQ(testLine, gdLine);

  // we have line spanning positive X-axis, does not need to be clipped
  origLine.Set(1, 0, 0, 999, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, 0, 999, 0, 0);
  ASSERT_EQ(testLine, gdLine);

  // we have line spanning X-axis, need to be clipped
  origLine.Set(-999, 0, 0, 999, 0, 0);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(0, 0, 0, 999, 0, 0);
  ASSERT_EQ(testLine, gdLine);

  // we have line spanning Z-axis, should be clipped
  origLine.Set(1, 0, -999, 1, 0, 999);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, 0, -1, 1, 0, 1);
  ASSERT_EQ(testLine, gdLine);

  // we have line behind camera, should not be added to fieldLines
  origLine.Set(-1, -999, 0, -1, 999, 0);
  ASSERT_FALSE(UpdateLine_Test(agent, origLine, testLine));

  // we have diagonal line that needs to be clipped on both Z and Y axis
  origLine.Set(1, -999, -999, 1, 999, 999);
  UpdateLine_Test(agent, origLine, testLine);
  gdLine.Set(1, -1, -1, 1, 1, 1);
  ASSERT_EQ(testLine, gdLine);
}

/// \Brief Test whether update landmarks and transformation from global to
/// local works
TEST_F(PerceptorTest, Perceptor_UpdateLandmark_Norestrictvis)
{
  math::Vector3<double> origLandmark, gdLandmark, testLandmark;
  GameState::restrictVision = false;
  perceptor->useNoise = false;
  gameState->AddAgent(1, "blue");
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);

  /// agent's camera is at origin, facing straight down the positive y-axis
  agent.pos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, M_PI / 2);
  perceptor->SetG2LMat(agent);

  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(0, -1, 0);
  ASSERT_EQ(gdLandmark, testLandmark);

  origLandmark.Set(1, 1, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(1, -1, 0);
  ASSERT_EQ(gdLandmark, testLandmark);

  agent.cameraRot.Euler(0, 0, M_PI / 4);
  perceptor->SetG2LMat(agent);
  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark.Set(1 / sqrt(2), -1 / sqrt(2), 0);
  ASSERT_EQ(gdLandmark, testLandmark);

  agent.pos.Set(1, 2, 3);
  perceptor->SetG2LMat(agent);
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
  perceptor->SetViewFrustrum();
  perceptor->useNoise = false;
  gameState->AddAgent(1, "blue");
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);

  agent.pos.Set(0, 0, 0);
  agent.cameraRot.Euler(0, 0, 0);
  perceptor->SetG2LMat(agent);

  origLandmark.Set(-999, 0, 0);
  ASSERT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(-1, 0, 0);
  ASSERT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(0, 999, 0);
  ASSERT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(0, 0, -999);
  ASSERT_FALSE(UpdateLandmark_Test(agent, origLandmark, testLandmark));

  origLandmark.Set(2, 1, 1);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark = origLandmark;
  ASSERT_EQ(gdLandmark, testLandmark);

  origLandmark.Set(1, 0, 0);
  UpdateLandmark_Test(agent, origLandmark, testLandmark);
  gdLandmark = origLandmark;
  ASSERT_EQ(gdLandmark, testLandmark);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
