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
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/

#include <ignition/math.hh>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "gtest/gtest.h"
#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/BeforeKickOffState.hh"
#include "robocup3ds/states/CornerKickState.hh"
#include "robocup3ds/states/FreeKickState.hh"
#include "robocup3ds/states/GameOverState.hh"
#include "robocup3ds/states/GoalKickState.hh"
#include "robocup3ds/states/GoalState.hh"
#include "robocup3ds/states/KickInState.hh"
#include "robocup3ds/states/KickOffState.hh"
#include "robocup3ds/states/PlayOnState.hh"
#include "robocup3ds/states/State.hh"

using namespace ignition;
using namespace states;

/// \class GameStateTest_basic
/// \brief This test fixture sets up a gameState object and two empty teams
class GameStateTest_basic : public ::testing::Test
{
  protected:
    virtual void SetUp()
    {
      GameState::useCounterForGameTime = true;
    }

  protected:
    virtual void TearDown() {}

  protected:
    GameState gameState;
    // green is third team and shouldnt be added
  protected:
    std::string teamNames[3] = {"blue", "red", "green"};
};

/// \brief Test whether GameState constructor and destructor works
TEST_F(GameStateTest_basic, GameState_construct_delete)
{
  SUCCEED();
}

/// \brief Test if configurations are loaded correctly
TEST_F(GameStateTest_basic, GameState_LoadConfiguration)
{
  std::map<std::string, std::string> config;

  config["gamestate_secondsfullgame"] = "400";
  config["gamestate_secondseachhalf"] = "500";
  gameState.LoadConfiguration(config);
  EXPECT_DOUBLE_EQ(GameState::SecondsFullGame, 400.0);
  EXPECT_DOUBLE_EQ(GameState::SecondsEachHalf, 200.0);

  config.clear();
  config["gamestate_secondseachhalf"] = "500";
  gameState.LoadConfiguration(config);
  EXPECT_DOUBLE_EQ(GameState::SecondsFullGame, 1000.0);
  EXPECT_DOUBLE_EQ(GameState::SecondsEachHalf, 500.0);

  config["gamestate_secondsgoalpause"] = "999";
  gameState.LoadConfiguration(config);
  EXPECT_DOUBLE_EQ(GameState::SecondsGoalPause, 999.0);
  config["gamestate_secondsgoalpause"] = "52gvs";
  EXPECT_DOUBLE_EQ(GameState::SecondsGoalPause, 999.0);

  config["gamestate_usecounterforgametime"] = "false";
  gameState.LoadConfiguration(config);
  EXPECT_FALSE(GameState::useCounterForGameTime);
  config["gamestate_usecounterforgametime"] = "true";
  gameState.LoadConfiguration(config);
  EXPECT_TRUE(GameState::useCounterForGameTime);
  config["gamestate_usecounterforgametime"] = "tru5245e";
  gameState.LoadConfiguration(config);
  EXPECT_TRUE(GameState::useCounterForGameTime);

  config["gamestate_playerlimit"] = "50";
  gameState.LoadConfiguration(config);
  EXPECT_EQ(GameState::playerLimit, 50);
  config["gamestate_playerlimit"] = "50.5";
  gameState.LoadConfiguration(config);
  EXPECT_EQ(GameState::playerLimit, 50);

  config.clear();
  config["gamestate_playerlimit"] = "11";
  config["gamestate_usecounterforgametime"] = "true";
  config["gamestate_secondsgoalpause"] = "3";
  config["gamestate_secondsfullgame"] = "600";
  gameState.LoadConfiguration(config);
}

/// \brief Test for adding teams and agents
TEST_F(GameStateTest_basic, GameState_add_teams_agents)
{
  // cannot add agent in incorrect play mode
  gameState.SetCurrent(gameState.playOnState);
  EXPECT_FALSE(gameState.AddAgent(0, "red"));
  gameState.SetCurrent(gameState.beforeKickOffState);

  // make sure that agents with bad unums or teams cannot be added
  for (int i = 0; i < 3; ++i)
  {
    for (int j = 0; j < 15; ++j)
    {
      bool returnValue = gameState.AddAgent(j + 1, teamNames[i]);
      if (i >= 2 || j + 1 >= 12)
      {
        EXPECT_FALSE(returnValue);
      }
      else
      {
        EXPECT_TRUE(returnValue);
      }
    }
  }

  // both teams are full so this should not work
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 2; ++j)
    {
      EXPECT_FALSE(gameState.AddAgent(j + 1, teamNames[i]));
    }
  }

  // make sure that there are only two teams and that each
  // team is initialized correctly
  EXPECT_EQ(gameState.teams.size(), 2u);
  for (int i = 0; i < 2; ++i)
  {
    EXPECT_EQ(gameState.teams.at(i)->members.size(), 11u);
    if (i == 0)
    {
      EXPECT_EQ(gameState.teams.at(i)->name, "blue");
      EXPECT_EQ(gameState.teams.at(i)->side, Team::Side::LEFT);
    }
    else
    {
      EXPECT_EQ(gameState.teams.at(i)->name, "red");
      EXPECT_EQ(gameState.teams.at(i)->side, Team::Side::RIGHT);
    }
    for (int j = 0; j < 11; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j);
      EXPECT_EQ(agent.uNum, j + 1);
      EXPECT_TRUE(agent.team != NULL);
    }
  }
}

/// \brief Test for removing teams and agents
TEST_F(GameStateTest_basic, GameState_remove_agents)
{
  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      gameState.AddAgent(j + 1, teamNames[i]);
    }
  }

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      bool returnValue = gameState.RemoveAgent(j + 1, teamNames[i]);
      EXPECT_TRUE(returnValue);
      EXPECT_EQ(gameState.teams.at(i)->members.size(),
                11u - static_cast<size_t>(j + 1));
      for (auto &agent : gameState.teams.at(i)->members)
      {
        EXPECT_NE(agent.uNum, j + 1);
      }
    }
  }

  for (int i = 0; i < 2; ++i)
  {
    for (int j = 0; j < 11; ++j)
    {
      bool returnValue = gameState.RemoveAgent(j + 1, teamNames[i]);
      EXPECT_FALSE(returnValue);
      EXPECT_EQ(gameState.teams.at(i)->members.size(), 0u);
    }
  }
}

/// \brief Test for add agent with uNum 0, which assigns it the next free uNum
TEST_F(GameStateTest_basic, GameState_add_agent_0)
{
  for (int i = 0; i < 2; i++)
  {
    EXPECT_TRUE(gameState.AddAgent(0, teamNames[i]));
    EXPECT_EQ(gameState.teams[i]->members.at(0).uNum, 1);
    EXPECT_FALSE(gameState.AddAgent(1, teamNames[i]));

    EXPECT_TRUE(gameState.AddAgent(2, teamNames[i]));
    EXPECT_EQ(gameState.teams[i]->members.at(1).uNum, 2);

    EXPECT_TRUE(gameState.AddAgent(0, teamNames[i]));
    EXPECT_EQ(gameState.teams[i]->members.at(2).uNum, 3);
    EXPECT_FALSE(gameState.AddAgent(3, teamNames[i]));

    EXPECT_TRUE(gameState.AddAgent(0, teamNames[i]));
    EXPECT_EQ(gameState.teams[i]->members.at(3).uNum, 4);
    EXPECT_FALSE(gameState.AddAgent(4, teamNames[i]));

    EXPECT_EQ(gameState.teams.at(i)->members.size(), 4u);

    for (int j = 0; j < 7; j++)
    {
      EXPECT_TRUE(gameState.AddAgent(0, teamNames[i]));
    }

    EXPECT_FALSE(gameState.AddAgent(0, teamNames[i]));
    EXPECT_EQ(gameState.teams.at(i)->members.size(), 11u);
  }
}

/// \brief Test for whether the move ball functions are working as intended
TEST_F(GameStateTest_basic, GameState_move_ball)
{
  math::Vector3<double> pos(15, 10, SoccerField::BallRadius);
  gameState.MoveBall(pos);
  EXPECT_EQ(pos, gameState.GetBall());

  gameState.MoveBallToCenter();
  EXPECT_EQ(SoccerField::BallCenterPosition, gameState.GetBall());

  pos.Set(-10, 5, SoccerField::BallRadius);
  gameState.MoveBall(pos);
  gameState.MoveBallForGoalKick();
  EXPECT_EQ(math::Vector3<double>(-SoccerField::HalfFieldWidth + 1, 0,
                                  SoccerField::BallRadius),
            gameState.GetBall());

  pos.Set(10, -5, SoccerField::BallRadius);
  gameState.MoveBall(pos);
  gameState.MoveBallForGoalKick();
  EXPECT_EQ(math::Vector3<double>(SoccerField::HalfFieldWidth - 1, 0,
                                  SoccerField::BallRadius),
            gameState.GetBall());


  std::vector<math::Vector3<double>> nearFourCorners;
  nearFourCorners.push_back(math::Vector3<double>(-1, -1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(-1, 1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(1, -1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(1, 1,
                            SoccerField::BallRadius));

  std::vector<math::Vector3<double>> fourCorners;
  fourCorners.push_back(math::Vector3<double>(-SoccerField::HalfFieldWidth,
                        -SoccerField::HalfFieldHeight,
                        SoccerField::BallRadius));
  fourCorners.push_back(math::Vector3<double>(-SoccerField::HalfFieldWidth,
                        SoccerField::HalfFieldHeight, SoccerField::BallRadius));
  fourCorners.push_back(math::Vector3<double>(SoccerField::HalfFieldWidth,
                        -SoccerField::HalfFieldHeight,
                        SoccerField::BallRadius));
  fourCorners.push_back(math::Vector3<double>(SoccerField::HalfFieldWidth,
                        SoccerField::HalfFieldHeight, SoccerField::BallRadius));

  for (size_t i = 0; i < nearFourCorners.size(); ++i)
  {
    gameState.MoveBall(nearFourCorners.at(i));
    gameState.MoveBallToCorner();
    EXPECT_EQ(fourCorners.at(i), gameState.GetBall());
  }

  std::vector<math::Vector3<double>> outOfBounds;
  outOfBounds.push_back(math::Vector3<double>(-17, -13, -5));
  outOfBounds.push_back(math::Vector3<double>(17, 13, 5));
  outOfBounds.push_back(math::Vector3<double>(11, -12, -5));
  outOfBounds.push_back(math::Vector3<double>(-11, 12, -5));

  std::vector<math::Vector3<double>> inBounds;
  inBounds.push_back(math::Vector3<double>(-SoccerField::HalfFieldWidth,
                     -SoccerField::HalfFieldHeight, SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(SoccerField::HalfFieldWidth,
                     SoccerField::HalfFieldHeight, SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(11, -SoccerField::HalfFieldHeight,
                     SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(-11, SoccerField::HalfFieldHeight,
                     SoccerField::BallRadius));

  for (size_t i = 0; i < outOfBounds.size(); ++i)
  {
    gameState.MoveBall(outOfBounds.at(i));
    gameState.MoveBallInBounds();
    EXPECT_EQ(inBounds.at(i), gameState.GetBall());
  }
}

/// \brief Test for whether the move agent functions are working as intended
TEST_F(GameStateTest_basic, GameState_move_agent)
{
  gameState.AddAgent(1, "blue");
  Agent &agent = gameState.teams.at(0)->members.at(0);
  math::Vector3<double> pos(15, 10, GameState::beamHeight);
  gameState.MoveAgent(agent, pos);
  EXPECT_EQ(pos, agent.pos);
  EXPECT_EQ(pos, gameState.teams.at(0)->members.at(0).pos);

  pos.Set(5, 6, GameState::beamHeight);
  math::Quaternion<double>rot(0, 0, 1.0);
  gameState.MoveAgent(agent, 5, 6, 1.0);
  EXPECT_EQ(agent.pos, pos);
  EXPECT_EQ(rot, agent.rot);
  EXPECT_EQ(pos, gameState.teams.at(0)->members.at(0).pos);
  EXPECT_EQ(rot, gameState.teams.at(0)->members.at(0).rot);

  pos.Set(0, 9, 0);
  gameState.MoveAgent(agent, pos);
  gameState.MoveAgentToSide(agent);
  pos.Set(0, -SoccerField::HalfFieldHeight, GameState::beamHeight);
  EXPECT_EQ(pos, agent.pos);

  pos.Set(0, -9, 0);
  gameState.MoveAgent(agent, pos);
  gameState.MoveAgentToSide(agent);
  pos.Set(0, SoccerField::HalfFieldHeight, GameState::beamHeight);
  EXPECT_EQ(pos, agent.pos);

  pos.Set(-5, -6, GameState::beamHeight);
  rot.Euler(1.0, 0.5, 0.7);
  gameState.MoveAgent(agent, pos, rot);
  EXPECT_EQ(agent.pos, pos);
  EXPECT_EQ(rot, agent.rot);
  EXPECT_EQ(pos, gameState.teams.at(0)->members.at(0).pos);
  EXPECT_EQ(rot, gameState.teams.at(0)->members.at(0).rot);

  pos.Set(7, 8, GameState::beamHeight);
  rot.Euler(0, 0, 1.25);
  for (int i = 0; i < 100; ++i)
  {
    bool result = gameState.BeamAgent(1, "blue", 7, 8, 1.25);
    EXPECT_TRUE(result);
    EXPECT_LE(agent.pos.Distance(pos), 0.15);
    EXPECT_LE(fabs(agent.rot.Euler().Z() - rot.Euler().Z()), 0.1);
  }
  EXPECT_FALSE(gameState.BeamAgent(2, "blue", 7, 8, 1.25));
  EXPECT_FALSE(gameState.BeamAgent(1, "red", 7, 8, 1.25));
  gameState.SetCurrent(gameState.playOnState);
  EXPECT_FALSE(gameState.BeamAgent(1, "blue", 7, 8, 1.25));
}

/// \class GameStateTest_fullTeams
/// \brief This test fixture sets up a gameState object and two full teams
class GameStateTest_fullTeams : public GameStateTest_basic
{
  protected:
    virtual void SetUp()
    {
      GameState::useCounterForGameTime = true;
      GameStateTest_basic::SetUp();
      for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 11; ++j)
        {
          gameState.AddAgent(j + 1, teamNames[i]);
        }
      }
    }

  protected:
    virtual void resetPositions()
    {
      for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 11; ++j)
        {
          Agent &agent = gameState.teams.at(i)->members.at(j);
          gameState.MoveAgent(agent, math::Vector3<double>
                              (0, 0, GameState::beamHeight));
        }
      }
    }

  protected:
    virtual void resetPositionsForKickOff()
    {
      for (int i = 0; i < 2; ++i)
      {
        for (int j = 0; j < 11; ++j)
        {
          Agent &agent = gameState.teams.at(i)->members.at(j);
          if (gameState.teams.at(i)->side == Team::Side::LEFT)
          {
            gameState.MoveAgent(agent, math::Vector3<double>
                                (-5, 0, GameState::beamHeight));
          }
          else
          {
            gameState.MoveAgent(agent, math::Vector3<double>
                                (5, 0, GameState::beamHeight));
          }
        }
      }
    }

    // protected:
    //   virtual void resetPositions(int _team)
    //   {
    //     if (_team != 0 || _team != 1)
    //     {
    //       return;
    //     }
    //     for (int j = 0; j < 11; ++j)
    //     {
    //       Agent &agent = gameState.teams.at(_team)->members.at(j);
    //       gameState.MoveAgent(agent, math::Vector3<double>
    //                           (0, 0, GameState::beamHeight));
    //     }
    //   }
};

/// \brief Test for whether beforeKickOff play mode transitions correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_beforeKickOff_kickOff)
{
  // try first half
  EXPECT_EQ(gameState.GetHalf(), GameState::Half::FIRST_HALF);
  while (gameState.GetGameTime() < GameState::SecondsBeforeKickOff + 1)
  {
    gameState.Update();
    if (gameState.GetGameTime() < GameState::SecondsBeforeKickOff)
    {
      EXPECT_EQ(gameState.GetBall(), SoccerField::BallCenterPosition);
      EXPECT_EQ(gameState.GetCurrentState()->name, "BeforeKickOff");
    }
    else
    {
      EXPECT_EQ(gameState.GetCurrentState()->name, "KickOffLeft");
    }
  }

  // try second half
  gameState.SetCycleCounter(0);
  gameState.SetCurrent(gameState.beforeKickOffState, true);
  gameState.SetHalf(GameState::Half::SECOND_HALF);
  EXPECT_EQ(gameState.GetHalf(), GameState::Half::SECOND_HALF);
  while (gameState.GetGameTime() < GameState::SecondsBeforeKickOff + 1)
  {
    gameState.Update();
    if (gameState.GetGameTime() < GameState::SecondsBeforeKickOff)
    {
      EXPECT_EQ(gameState.GetBall(), SoccerField::BallCenterPosition);
      EXPECT_EQ(gameState.GetCurrentState()->name, "BeforeKickOff");
    }
    else
    {
      EXPECT_EQ(gameState.GetCurrentState()->name, "KickOffRight");
    }
  }
}

/// \brief Test for whether KickOff play mode transitions correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_kickOff_playOn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.kickOffLeftState);
  states.push_back(gameState.kickOffRightState);

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  // Test for both left and right kick offs
  for (size_t i = 0; i < states.size(); ++i)
  {
    std::shared_ptr<State> state = states.at(i);

    // test for transition when KickOff times out after 15 secs
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(state);
    while (gameState.GetGameTime() < GameState::SecondsKickOff + 1)
    {
      gameState.Update();
      // cout << gameState.GetGameTime() << " " << gameState.
      // GetCurrentState()->name << " " << gameState.GetCurrentState()->
      // getElapsedTime() << endl;
      if (gameState.GetGameTime() < GameState::SecondsKickOff)
      {
        EXPECT_EQ(gameState.GetCurrentState()->name, state->GetName());
      }
      else
      {
        EXPECT_EQ(gameState.GetCurrentState()->name, "PlayOn");
      }
    }

    // test for transition when ball is touched
    gameState.SetCurrent(state);
    gameState.Update();
    std::shared_ptr<GameState::BallContact> ballContact(
      new GameState::BallContact(1, gameState.teams.at(i)->side,
                                 gameState.GetGameTime(),
                                 math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_TRUE(gameState.touchBallKickoff != NULL);
    EXPECT_EQ(gameState.touchBallKickoff, ballContact);
    EXPECT_EQ(gameState.GetCurrentState()->name, "PlayOn");
    EXPECT_EQ(gameState.GetLastSideTouchedBall(), gameState.teams.at(i)->side);

    // test for transition when double touching occurs
    gameState.SetCurrent(state);
    gameState.Update();
    std::shared_ptr<GameState::BallContact> ballContact2(
      new GameState::BallContact(1, gameState.teams.at(i)->side,
                                 gameState.GetGameTime(),
                                 math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact2);
    for (int j = 0; j < 10; ++j)
    {
      gameState.Update();
      EXPECT_EQ(gameState.GetCurrentState()->name, "PlayOn");
    }
    std::shared_ptr<GameState::BallContact> ballContact3(
      new GameState::BallContact(1, gameState.teams.at(i)->side,
                                 gameState.GetGameTime(),
                                 math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact3);
    gameState.Update();
    EXPECT_EQ(gameState.GetCurrentState()->name, states.at((i + 1) % 2)->name);

    // test that transition to other kickoff does not happen when
    // double touch does not occur
    gameState.SetCurrent(state);
    gameState.Update();
    std::shared_ptr<GameState::BallContact> ballContact4(
      new GameState::BallContact(1, gameState.teams.at(i)->side,
                                 gameState.GetGameTime(),
                                 math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact4);
    for (int j = 0; j < 10; ++j)
    {
      gameState.Update();
      if (j == 5)
      {
        std::shared_ptr<GameState::BallContact> ballContact5(
          new GameState::BallContact(
            5, gameState.teams.at((i + 1) % 2)->side,
            gameState.GetGameTime(),
            math::Vector3<double>::Zero));
        gameState.ballContactHistory.push_back(ballContact5);
      }
    }
    std::shared_ptr<GameState::BallContact> ballContact6(
      new GameState::BallContact(1, gameState.teams.at(i)->side,
                                 gameState.GetGameTime(),
                                 math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact6);
    gameState.Update();
    EXPECT_EQ(gameState.GetCurrentState()->name, "PlayOn");

    // test that ball contact history is cleaned up correctly
    gameState.ClearBallContactHistory();
    EXPECT_FALSE(gameState.GetLastBallContact());
    EXPECT_EQ(gameState.GetLastSideTouchedBall(),
              Team::Side::NEITHER);
  }
}

/// \brief Test for whether PlayOn play mode transitions to kickIn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_kickIn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.kickInLeftState);
  states.push_back(gameState.kickInRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  std::vector<math::Vector3<double>> ballPositions;
  ballPositions.push_back(math::Vector3<double>(0, -15,
                          SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(0, 15,
                          SoccerField::BallRadius));

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    for (size_t j = 0; j < ballPositions.size(); ++j)
    {
      gameState.MoveBall(math::Vector3<double>::Zero);
      gameState.SetCurrent(gameState.playOnState);
      gameState.Update();
      EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

      ballContact = std::shared_ptr<GameState::BallContact>(
                      new GameState::BallContact(1,
                          gameState.teams.at(i)->side,
                          gameState.GetGameTime(),
                          math::Vector3<double>::Zero));
      gameState.ballContactHistory.push_back(ballContact);

      gameState.MoveBall(ballPositions.at(j));
      gameState.Update();
      EXPECT_EQ(states.at((i + 1) % 2)->name,
                gameState.GetCurrentState()->name);
    }
  }
}

/// \brief Test for whether PlayOn play mode transitions to cornerKick correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_cornerKick)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.cornerKickLeftState);
  states.push_back(gameState.cornerKickRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  std::vector<math::Vector3<double>> ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 1),
                            5, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 1,
                            -5, SoccerField::BallRadius));

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    gameState.MoveBall(math::Vector3<double>::Zero);
    gameState.SetCurrent(gameState.playOnState);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.MoveBall(ballPositions.at(i));
    // cout << gameState.getLastBallContact()->side << " " <<
    // gameState.GetBall() << " " <<
    // gameState.teams.at(i)->side << endl;
    gameState.Update();
    EXPECT_EQ(states.at((i + 1) % 2)->name, gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether PlayOn play mode transitions to goal correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_goal)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.goalLeftState);
  states.push_back(gameState.goalRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  std::vector<math::Vector3<double>> ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 0.5),
                            1, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 0.5,
                            -1, SoccerField::BallRadius));

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    gameState.MoveBall(math::Vector3<double>::Zero);
    gameState.SetCurrent(gameState.playOnState);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    gameState.MoveBall(ballPositions.at(i));
    gameState.Update();
    EXPECT_EQ(states.at((i + 1) % 2)->name, gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether PlayOn play mode transitions to goalKick correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_goalKick)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.goalKickLeftState);
  states.push_back(gameState.goalKickRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  std::vector<math::Vector3<double>> ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 1),
                            5, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 1,
                            -5, SoccerField::BallRadius));

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    gameState.MoveBall(math::Vector3<double>::Zero);
    gameState.SetCurrent(gameState.playOnState);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1,
                        gameState.teams.at((i + 1) % 2)->side,
                        gameState.GetGameTime(),
                        math::Vector3<double>::Zero));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.MoveBall(ballPositions.at(i));
    gameState.Update();
    EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether goal play mode transitions to kickOff correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_goal_kickOff)
{
  std::vector<std::shared_ptr<State>> beforeStates;
  beforeStates.push_back(gameState.goalLeftState);
  beforeStates.push_back(gameState.goalRightState);

  std::vector<std::shared_ptr<State>> afterStates;
  afterStates.push_back(gameState.kickOffRightState);
  afterStates.push_back(gameState.kickOffLeftState);

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);

  for (size_t i = 0; i < beforeStates.size(); ++i)
  {
    // check that valid goal transitions correctly
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(beforeStates.at(i));
    gameState.teams.at(i)->canScore = true;

    while (gameState.GetGameTime() < GameState::SecondsGoalPause)
    {
      EXPECT_EQ(beforeStates.at(i)->name, gameState.GetCurrentState()->name);
      gameState.Update();
    }
    EXPECT_EQ(afterStates.at(i)->name, gameState.GetCurrentState()->name);

    gameState.SetCurrent(beforeStates.at(i));
    gameState.teams.at(i)->canScore = false;
    gameState.Update();
    EXPECT_EQ(afterStates.at(i)->name, gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether kickIn play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_kickIn_playOn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.kickInLeftState);
  states.push_back(gameState.kickInRightState);
  std::shared_ptr<GameState::BallContact> ballContact;

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    // transition from timing out
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickIn)
    {
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
      gameState.Update();
    }
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    // transition to play on from touching the ball
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickInPause)
    {
      gameState.Update();
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
    }
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether cornerKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_cornerKick_playOn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.cornerKickLeftState);
  states.push_back(gameState.cornerKickRightState);
  std::shared_ptr<GameState::BallContact> ballContact;

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    // transition from timing out
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickIn)
    {
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
      gameState.Update();
    }
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    // transition to play on from touching the ball
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickInPause)
    {
      gameState.Update();
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
    }
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether freeKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_freeKick_playOn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.freeKickLeftState);
  states.push_back(gameState.freeKickRightState);
  std::shared_ptr<GameState::BallContact> ballContact;

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    // transition from timing out
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickIn)
    {
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
      gameState.Update();
    }
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    // transition to play on from touching the ball
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickInPause)
    {
      gameState.Update();
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
    }
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether goalKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_goalKick_playOn)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.goalKickLeftState);
  states.push_back(gameState.goalKickRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  std::vector<math::Vector3<double>> ballPositions;
  ballPositions.push_back(math::Vector3<double>(-16, 5,
                          SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(16, -5,
                          SoccerField::BallRadius));

  EXPECT_EQ(Team::Side::LEFT, gameState.teams.at(0)->side);
  EXPECT_EQ(Team::Side::RIGHT, gameState.teams.at(1)->side);
  for (size_t i = 0; i < states.size(); ++i)
  {
    // transition from timing out
    gameState.SetCycleCounter(0);
    gameState.MoveBall(ballPositions.at(i));
    gameState.SetCurrent(states.at(i));

    while (gameState.GetGameTime() < GameState::SecondsKickIn)
    {
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
      gameState.Update();
    }
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);

    // transition to play on from touching the ball
    gameState.SetCycleCounter(0);
    gameState.SetCurrent(states.at(i));
    while (gameState.GetGameTime() < GameState::SecondsKickInPause)
    {
      gameState.Update();
      EXPECT_EQ(states.at(i)->name, gameState.GetCurrentState()->name);
    }
    // move ball out of penalty area
    gameState.MoveBallToCenter();
    gameState.Update();
    EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
  }
}

/// \brief Test for whether when running full game, game transitions to
/// kickOffRight at end of first half and to gameOver at the end of the
/// second half
TEST_F(GameStateTest_fullTeams, GameState_transition_checkTiming)
{
  double firstHalfTime = GameState::SecondsEachHalf +
                         GameState::SecondsBeforeKickOff;
  double firstHalfKickOffTime = GameState::SecondsBeforeKickOff +
                                GameState::SecondsKickOff;
  double secondHalfTime = GameState::SecondsBeforeKickOff +
                          GameState::SecondsFullGame;
  double secondHalfKickOffTime = GameState::SecondsKickOff + firstHalfTime;

  while (gameState.GetGameTime() < firstHalfTime)
  {
    EXPECT_TRUE(gameState.GetElapsedGameTime() < GameState::SecondsEachHalf);
    EXPECT_TRUE(gameState.GetHalf() == GameState::Half::FIRST_HALF);
    EXPECT_TRUE(gameState.teams.at(0)->side == Team::Side::LEFT
                && gameState.teams.at(1)->side ==
                Team::Side::RIGHT);
    if (gameState.GetGameTime() < GameState::SecondsBeforeKickOff)
    {
      EXPECT_EQ("BeforeKickOff", gameState.GetCurrentState()->name);
    }
    else if (gameState.GetGameTime() < firstHalfKickOffTime)
    {
      EXPECT_EQ("KickOffLeft", gameState.GetCurrentState()->name);
    }
    else
    {
      EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
    }
    gameState.Update();
  }
  EXPECT_EQ("KickOffRight", gameState.GetCurrentState()->name);
  while (gameState.GetGameTime() < secondHalfTime)
  {
    EXPECT_TRUE(gameState.GetHalf() == GameState::Half::SECOND_HALF);
    EXPECT_TRUE(gameState.teams.at(0)->side == Team::Side::RIGHT
                && gameState.teams.at(1)->side ==
                Team::Side::LEFT);
    EXPECT_TRUE(gameState.GetElapsedGameTime() < GameState::SecondsEachHalf);
    if (gameState.GetGameTime() < secondHalfKickOffTime)
    {
      EXPECT_EQ("KickOffRight", gameState.GetCurrentState()->name);
    }
    else
    {
      EXPECT_EQ("PlayOn", gameState.GetCurrentState()->name);
    }
    gameState.Update();
  }
  for (int i = 0; i < 50; i++)
  {
    EXPECT_EQ("GameOver", gameState.GetCurrentState()->name);
    gameState.Update();
  }
}

/// \brief Test to check whether the GameState CheckCanScore function
/// works as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckCanScore)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.kickOffLeftState);
  states.push_back(gameState.kickOffRightState);
  std::shared_ptr<GameState::BallContact> ballContact;
  math::Vector3<double> pos;

  for (size_t i = 0; i < states.size(); ++i)
  {
    // case 1: kickoff agent touches ball, teammate touches ball outside circle
    // team should be able to score
    gameState.SetCurrent(states.at(i));
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    pos.Set(5, 5, SoccerField::BallRadius);
    ballContact = std::shared_ptr<GameState::BallContact>(new
                  GameState::BallContact(2, gameState.teams.at(i)->side,
                                         gameState.GetGameTime(), pos));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_TRUE(gameState.teams.at(i)->canScore);

    // case 2: kickoff agent touches ball, teammate touches ball inside circle
    // team should not be able to score
    gameState.SetCurrent(states.at(i));
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      2, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);

    // case 3: kickoff agent touches ball, and touches it again afterwards
    // team should not be able to score
    gameState.SetCurrent(states.at(i));
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);

    // case 4: kickoff agent touches ball, and
    // someone on opposing team touches it
    // team should be able to score
    gameState.SetCurrent(states.at(i));
    gameState.Update();
    EXPECT_FALSE(gameState.teams.at(i)->canScore);
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(
                      1, gameState.teams.at(i)->side,
                      gameState.GetGameTime(),
                      gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    ballContact = std::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1,
                        gameState.teams.at((i + 1) % 2)->side,
                        gameState.GetGameTime(),
                        gameState.GetBall()));
    gameState.ballContactHistory.push_back(ballContact);
    gameState.Update();
    EXPECT_TRUE(gameState.teams.at(i)->canScore);
  }
}

/// \brief Test to check whether the CheckIllegalDefense
/// function works as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckIllegalDefense)
{
  for (int i = 0; i < 2; ++i)
  {
    math::Box penaltyBox;
    math::Vector3<double> goalCenter;
    if (i == 0)
    {
      penaltyBox = SoccerField::PenaltyBoxLeft;
      goalCenter = SoccerField::GoalCenterLeft;
    }
    else
    {
      penaltyBox = SoccerField::PenaltyBoxRight;
      goalCenter = SoccerField::GoalCenterRight;
    }
    math::Vector3<double> penaltyPos = penaltyBox.Center();
    gameState.SetCurrent(gameState.playOnState);

    // test whether fourth agent in penalty box gets beamed out
    resetPositions();
    gameState.Update();
    for (int j = 0; j < 3; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j + 1);
      gameState.MoveAgent(agent, penaltyPos);
    }
    gameState.Update();
    for (int j = 0; j < 3; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j + 1);
      EXPECT_TRUE(penaltyBox.Contains(agent.pos));
    }
    Agent &agent2 = gameState.teams.at(i)->members.at(4);
    gameState.MoveAgent(agent2, penaltyPos);
    gameState.Update();
    EXPECT_FALSE(penaltyBox.Contains(agent2.pos));

    // test whether goalie (if fourth goalie) stays in penalty box and farthest
    // agent gets beamed out
    resetPositions();
    gameState.Update();
    for (int j = 1; j < 4; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j);
      if (j == 3)
      {
        // this agent is farthest away from goal
        gameState.MoveAgent(agent, penaltyPos);
      }
      else
      {
        gameState.MoveAgent(agent, goalCenter);
      }
    }
    gameState.Update();
    Agent &agent3 = gameState.teams.at(i)->members.at(0);
    gameState.MoveAgent(agent3, penaltyPos);
    gameState.Update();
    EXPECT_TRUE(penaltyBox.Contains(agent3.pos));
    for (int j = 1; j < 4; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j);
      if (j == 3)
      {
        // this agent is farthest away from goal
        EXPECT_FALSE(penaltyBox.Contains(agent.pos));
      }
      else
      {
        EXPECT_TRUE(penaltyBox.Contains(agent.pos));
      }
    }

    // test that nothing happens if opponent also goes into goal box
    resetPositions();
    gameState.Update();
    for (int j = 0; j < 3; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j);
      gameState.MoveAgent(agent, penaltyPos);
    }
    gameState.Update();
    Agent &agent4 =
      gameState.teams.at((i + 1) % 2)->members.at(1);
    gameState.MoveAgent(agent4, penaltyPos);
    gameState.Update();
    for (int j = 0; j < 3; ++j)
    {
      Agent &agent = gameState.teams.at(i)->members.at(j);
      EXPECT_TRUE(penaltyBox.Contains(agent.pos));
    }
  }
}

/// \brief Test to check whether the CheckCrowding function
/// in gameState is working as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckCrowding)
{
  math::Vector3<double> testBallPos(5, 0, SoccerField::BallRadius);
  math::Vector3<double> crowdingEnablePos(5 + 0.5 *
                                          GameState::crowdingEnableRadius, 0,
                                          SoccerField::BallRadius);
  math::Vector3<double> innerRadius(5 + 0.5 * GameState::innerCrowdingRadius,
                                    0, SoccerField::BallRadius);
  math::Vector3<double> innerRadius2(5 + 0.75 *
                                     GameState::innerCrowdingRadius, 0,
                                     SoccerField::BallRadius);
  math::Vector3<double> outerRadius(5 + 0.5 * GameState::outerCrowdingRadius,
                                    0, SoccerField::BallRadius);
  math::Vector3<double> outerRadius2(5 + 0.75 *
                                     GameState::outerCrowdingRadius, 0,
                                     SoccerField::BallRadius);

  gameState.SetCurrent(gameState.playOnState);
  gameState.MoveBall(testBallPos);
  // move enemy agent close to ball to enable crowding

  for (int i = 0; i < 2; ++i)
  {
    resetPositions();
    // put agent on other team within crowding radius to ensure that crowding
    // rules are enabled
    Agent &agent =
      gameState.teams.at((i + 1) % 2)->members.at(1);
    gameState.MoveAgent(agent, crowdingEnablePos);
    gameState.Update();
    EXPECT_EQ(agent.pos, crowdingEnablePos);

    // test for whether inner radius crowding check works
    Agent &agent2 = gameState.teams.at(i)->members.at(1);
    gameState.MoveAgent(agent2, innerRadius);
    gameState.Update();
    EXPECT_EQ(agent2.pos, innerRadius);
    Agent &agent3 = gameState.teams.at(i)->members.at(2);
    gameState.MoveAgent(agent3, innerRadius2);
    gameState.Update();
    EXPECT_GE(agent3.pos.Distance(gameState.GetBall()),
              GameState::innerCrowdingRadius);

    // test for whether the outer radius crowding check works
    Agent &agent4 = gameState.teams.at(i)->members.at(3);
    gameState.MoveAgent(agent4, outerRadius);
    gameState.Update();
    EXPECT_EQ(agent4.pos, outerRadius);
    Agent &agent5 = gameState.teams.at(i)->members.at(4);
    gameState.MoveAgent(agent5, outerRadius2);
    gameState.Update();
    EXPECT_GE(agent5.pos.Distance(gameState.GetBall()),
              GameState::outerCrowdingRadius);
  }
}

/// \brief Test to check whether the CheckImmobility function in gameState is
/// working as intended
TEST_F(GameStateTest_basic, GameState_CheckImmobilityFallen)
{
  math::Vector3<double> pos(0, 0, GameState::beamHeight);
  std::vector<math::Vector3<double>> fallenPos;
  fallenPos.push_back(math::Vector3<double>(0.0, 0.0, 0.1));
  fallenPos.push_back(math::Vector3<double>(0.1, 0.1, 0.1));
  gameState.AddAgent(1, "blue");
  gameState.SetCurrent(gameState.playOnState);

  // check immobility and fallen for goalie
  Agent &agent = gameState.teams.at(0)->members.at(0);
  agent.pos = agent.prevPos = pos;
  while (gameState.GetGameTime() < 2 * GameState::immobilityTimeLimit)
  {
    EXPECT_EQ(agent.pos, pos);
    gameState.Update();
  }
  EXPECT_NE(agent.pos, pos);

  gameState.SetCycleCounter(0);
  int c = -1;
  while (gameState.GetGameTime() < 2 * GameState::fallenTimeLimit)
  {
    c++;
    gameState.MoveAgent(agent, fallenPos.at(c % 2));
    EXPECT_EQ(agent.pos, fallenPos.at(c % 2));
    gameState.Update();
  }
  EXPECT_NE(agent.pos, fallenPos.at(c % 2));

  // check immobility and fallen for non-goalie
  gameState.SetCycleCounter(0);
  gameState.SetCurrent(gameState.beforeKickOffState);
  gameState.AddAgent(2, "blue");
  gameState.SetCurrent(gameState.playOnState);
  Agent &agent2 = gameState.teams.at(0)->members.at(1);
  agent2.pos = agent2.prevPos = pos;
  while (gameState.GetGameTime() < GameState::immobilityTimeLimit)
  {
    EXPECT_EQ(agent2.pos, pos);
    gameState.Update();
  }
  EXPECT_NE(agent2.pos, pos);

  gameState.SetCycleCounter(0);
  c = -1;
  while (gameState.GetGameTime() < GameState::fallenTimeLimit)
  {
    c++;
    gameState.MoveAgent(agent2, fallenPos.at(c % 2));
    EXPECT_EQ(agent2.pos, fallenPos.at(c % 2));
    gameState.Update();
  }
  EXPECT_NE(agent2.pos, fallenPos.at(c % 2));
}

/// \brief Test to check whether the DropBallImpl function in gameState is
/// working as intended
TEST_F(GameStateTest_fullTeams, GameState_DropBall)
{
  gameState.MoveBallToCenter();

  for (int i = 0; i < 2; ++i)
  {
    resetPositions();
    std::shared_ptr<Team> allowedTeam = gameState.teams.at(i);
    std::shared_ptr<Team> notAllowedTeam =
      gameState.teams.at((i + 1) % 2);
    gameState.DropBallImpl(allowedTeam->side);

    for (int j = 0; j < 11; ++j)
    {
      Agent &agent = notAllowedTeam->members.at(j);
      EXPECT_GE(agent.pos.Distance(gameState.GetBall()),
                GameState::dropBallRadius);
    }
  }
}

/// \brief Test to check whether the CheckGoalKickIllegalDefense
/// function in gameState is working as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckGoalKickIllegalDefense)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.goalKickLeftState);
  states.push_back(gameState.goalKickRightState);

  std::vector<math::Vector3<double>> inPenaltyBox;
  inPenaltyBox.push_back(math::Vector3<double>(
                           -SoccerField::HalfFieldWidth + 1, 0,
                           GameState::beamHeight));
  inPenaltyBox.push_back(math::Vector3<double>(
                           SoccerField::HalfFieldWidth - 1, 0,
                           GameState::beamHeight));

  std::vector<math::Box> penaltyBox;
  penaltyBox.push_back(SoccerField::PenaltyBoxLeft);
  penaltyBox.push_back(SoccerField::PenaltyBoxRight);

  for (size_t i = 0; i < states.size(); ++i)
  {
    gameState.MoveBall(inPenaltyBox.at(i));
    gameState.SetCurrent(states.at(i));
    while (!states.at(i)->hasInitialized)
    {
      gameState.Update();
    }
    Agent &enemyAgent =
      gameState.teams.at((i + 1) % 2)->members.at(0);
    gameState.MoveAgent(enemyAgent, inPenaltyBox.at(i));
    EXPECT_TRUE(penaltyBox.at(i).Contains(enemyAgent.pos));
    gameState.Update();
    EXPECT_FALSE(penaltyBox.at(i).Contains(enemyAgent.pos));
  }
}

/// \brief Test to check whether the CheckOffSidesOnKickOff
/// function in gameState is working as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckOffSidesOnKickOff)
{
  std::vector<std::shared_ptr<State>> states;
  states.push_back(gameState.kickOffLeftState);
  states.push_back(gameState.kickOffRightState);
  EXPECT_EQ(gameState.teams.at(0)->side, Team::Side::LEFT);
  EXPECT_EQ(gameState.teams.at(1)->side, Team::Side::RIGHT);
  for (size_t i = 0; i < states.size(); ++i)
  {
    Agent &ourAgent = gameState.teams.at(i)->members.at(0);
    Agent &theirAgent = gameState.teams.at(
                          (i + 1) % 2)->members.at(0);

    resetPositionsForKickOff();
    gameState.SetCurrent(states.at(i));
    gameState.Update();

    // check that all agent positions are unchanged
    for (int k = 0; k < 2; ++k)
    {
      for (int j = 0; j < 11; ++j)
      {
        Agent &agent = gameState.teams.at(k)->members.at(j);
        if (k == 0)
        {
          EXPECT_EQ(agent.pos, math::Vector3<double>(-5, 0,
                    GameState::beamHeight));
        }
        else
        {
          EXPECT_EQ(agent.pos, math::Vector3<double>(5, 0,
                    GameState::beamHeight));
        }
      }
    }

    // Ensure that we can move our agent into circle and nothing happens
    math::Vector3<double> pos(0, 0, GameState::beamHeight);
    gameState.MoveAgent(ourAgent, pos);
    gameState.Update();
    EXPECT_EQ(ourAgent.pos, pos);
    if (i == 0)
    { pos.Set(1, 0, GameState::beamHeight); }
    else
    { pos.Set(-1, 0, GameState::beamHeight); }
    gameState.MoveAgent(ourAgent, pos);
    gameState.Update();
    EXPECT_EQ(ourAgent.pos, pos);

    // Ensure that our agent is beamed back to its own side
    // if it violate sides and is not in circle
    if (i == 0)
    { pos.Set(1, 4, GameState::beamHeight); }
    else
    { pos.Set(-1, 4, GameState::beamHeight); }
    gameState.MoveAgent(ourAgent, pos);
    gameState.Update();
    if (i == 0)
    { EXPECT_LT(ourAgent.pos.X(), 0); }
    else
    { EXPECT_GT(ourAgent.pos.X(), 0); }

    // Ensure that enemy agent in circle is moved back to its own side
    pos.Set(0, 0, GameState::beamHeight);
    gameState.MoveAgent(theirAgent, pos);
    gameState.Update();
    EXPECT_NE(theirAgent.pos, pos);
    EXPECT_GE(theirAgent.pos.Distance(pos), SoccerField::CenterCircleRadius);
    if (i == 0)
    { pos.Set(1, 0, GameState::beamHeight); }
    else
    { pos.Set(-1, 0, GameState::beamHeight); }
    gameState.MoveAgent(theirAgent, pos);
    gameState.Update();
    EXPECT_NE(theirAgent.pos, pos);
    pos.Set(0, 0, GameState::beamHeight);
    EXPECT_GE(theirAgent.pos.Distance(pos), SoccerField::CenterCircleRadius);

    // Ensure that enemy agent is beamed back to its own side
    // if it violate sides
    if (i == 0)
    { pos.Set(-1, 4, GameState::beamHeight); }
    else
    { pos.Set(1, 4, GameState::beamHeight); }
    gameState.MoveAgent(theirAgent, pos);
    gameState.Update();
    if (i == 0)
    { EXPECT_GT(theirAgent.pos.X(), 0); }
    else
    { EXPECT_LT(theirAgent.pos.X(), 0); }
  }
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


