/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may !use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law || agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES || CONDITIONS OF ANY KIND, either express || implied.
 * See the License for the specific language governing permissions &&
 * limitations under the License.
 *
*/
#include <boost/scoped_ptr.hpp>
#include <string>
#include <vector>
#include "gtest/gtest.h"
#include "ignition/math.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"

using namespace std;
using namespace ignition;

class GameStateTest_basic : public ::testing::Test {
  protected:
  GameState *gameState;
  // green is third team && shouldnt be added
  string teamNames[3] = {"blue", "red", "green"};

  virtual void SetUp() {
    gameState = new GameState();
  }

  virtual void TearDown() {
    delete gameState;
  }
};

/// \brief Test whether GameState constructor && destructor works
TEST_F(GameStateTest_basic, GameState_construct_delete) {
  SUCCEED();
}

/// \brief Test for adding teams && agents
TEST_F(GameStateTest_basic, GameState_add_teams_agents) {
  // make sure that agents with bad unums || teams cannot be added
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 15; j++) {
      bool returnValue = gameState->addAgent(j + 1, teamNames[i]);
      if (i >= 2 || j + 1 >= 12) {
        ASSERT_FALSE(returnValue);
      }
      else
      {
        ASSERT_TRUE(returnValue);
      }
    }
  }

  // both teams are full so this should !work
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      ASSERT_FALSE(gameState->addAgent(j + 1, teamNames[i]));
    }
  }

  // make sure that their are only two teams && that each
  // team is initialized correctly
  ASSERT_EQ(gameState->teams.size(), 2u);
  for (int i = 0; i < 2; i++) {
    ASSERT_EQ(gameState->teams.at(i)->members.size(), 11u);
    if (i == 0) {
      ASSERT_EQ(gameState->teams.at(i)->name, "blue");
      ASSERT_EQ(gameState->teams.at(i)->side, GameState::Team::LEFT);
    }
    else
    {
      ASSERT_EQ(gameState->teams.at(i)->name, "red");
      ASSERT_EQ(gameState->teams.at(i)->side, GameState::Team::RIGHT);
    }
    for (int j = 0; j < 11; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j);
      ASSERT_EQ(agent.uNum, j + 1);
      ASSERT_TRUE(agent.team != NULL);
    }
  }
}

/// \brief Test for removing teams && agents
TEST_F(GameStateTest_basic, GameState_remove_agents) {
  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 11; j++) {
      gameState->addAgent(j + 1, teamNames[i]);
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 11; j++) {
      bool returnValue = gameState->removeAgent(j + 1, teamNames[i]);
      ASSERT_TRUE(returnValue);
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 11; j++) {
      bool returnValue = gameState->removeAgent(j + 1, teamNames[i]);
      ASSERT_FALSE(returnValue);
    }
  }

  for (int i = 0; i < 2; i++) {
    ASSERT_EQ(gameState->teams.at(i)->members.size(), 0u);
  }
}

/// \brief Test for add agent with uNum 0, which assigns it the next free uNum
TEST_F(GameStateTest_basic, GameState_add_agent_0) {
  ASSERT_TRUE(gameState->addAgent(0, teamNames[1]));
  ASSERT_EQ(gameState->teams[0]->members.at(0).uNum, 1);
  ASSERT_FALSE(gameState->addAgent(1, teamNames[1]));
  ASSERT_TRUE(gameState->addAgent(2, teamNames[1]));
  ASSERT_TRUE(gameState->addAgent(0, teamNames[1]));
  ASSERT_TRUE(gameState->addAgent(0, teamNames[1]));
  ASSERT_FALSE(gameState->addAgent(3, teamNames[1]));
  ASSERT_FALSE(gameState->addAgent(4, teamNames[1]));
  ASSERT_EQ(gameState->teams.at(0)->members.size(), 4u);
}

/// \brief Test for whether the move ball functions are working as intended
TEST_F(GameStateTest_basic, GameState_move_ball) {
  math::Vector3<double> pos(15, 10, SoccerField::BallRadius);
  gameState->MoveBall(pos);
  ASSERT_EQ(pos, gameState->GetBall());

  gameState->MoveBallToCenter();
  ASSERT_EQ(SoccerField::BallCenterPosition, gameState->GetBall());

  pos.Set(-10, 5, SoccerField::BallRadius);
  gameState->MoveBall(pos);
  gameState->MoveBallForGoalKick();
  ASSERT_EQ(math::Vector3<double>(-SoccerField::HalfFieldWidth + 1, 0,
                                  SoccerField::BallRadius),
            gameState->GetBall());

  pos.Set(10, -5, SoccerField::BallRadius);
  gameState->MoveBall(pos);
  gameState->MoveBallForGoalKick();
  ASSERT_EQ(math::Vector3<double>(SoccerField::HalfFieldWidth - 1, 0,
                                  SoccerField::BallRadius),
            gameState->GetBall());


  vector<math::Vector3<double> > nearFourCorners;
  nearFourCorners.push_back(math::Vector3<double>(-1, -1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(-1, 1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(1, -1,
                            SoccerField::BallRadius));
  nearFourCorners.push_back(math::Vector3<double>(1, 1,
                            SoccerField::BallRadius));

  vector<math::Vector3<double> > fourCorners;
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

  for (size_t i = 0; i < nearFourCorners.size(); i++) {
    gameState->MoveBall(nearFourCorners.at(i));
    gameState->MoveBallToCorner();
    ASSERT_EQ(fourCorners.at(i), gameState->GetBall());
  }

  vector<math::Vector3<double> > outOfBounds;
  outOfBounds.push_back(math::Vector3<double>(-17, -13, -5));
  outOfBounds.push_back(math::Vector3<double>(17, 13, 5));
  outOfBounds.push_back(math::Vector3<double>(11, -12, -5));
  outOfBounds.push_back(math::Vector3<double>(-11, 12, -5));

  vector<math::Vector3<double> > inBounds;
  inBounds.push_back(math::Vector3<double>(-SoccerField::HalfFieldWidth,
                     -SoccerField::HalfFieldHeight, SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(SoccerField::HalfFieldWidth,
                     SoccerField::HalfFieldHeight, SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(11, -SoccerField::HalfFieldHeight,
                     SoccerField::BallRadius));
  inBounds.push_back(math::Vector3<double>(-11, SoccerField::HalfFieldHeight,
                     SoccerField::BallRadius));

  for (size_t i = 0; i < outOfBounds.size(); i++) {
    gameState->MoveBall(outOfBounds.at(i));
    gameState->MoveBallInBounds();
    ASSERT_EQ(inBounds.at(i), gameState->GetBall());
  }
}

TEST_F(GameStateTest_basic, GameState_move_agent) {
  gameState->addAgent(1, "blue");
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);
  math::Vector3<double> pos(15, 10, GameState::beamHeight);
  gameState->MoveAgent(agent, pos);
  ASSERT_EQ(pos, agent.pos);
  ASSERT_EQ(pos, gameState->teams.at(0)->members.at(0).pos);

  pos.Set(5, 6, GameState::beamHeight);
  math::Quaternion<double>rot(0, 0, 1.0);
  gameState->MoveAgent(agent, 5, 6, 1.0);
  ASSERT_EQ(agent.pos, pos);
  ASSERT_EQ(rot, agent.rot);
  ASSERT_EQ(pos, gameState->teams.at(0)->members.at(0).pos);
  ASSERT_EQ(rot, gameState->teams.at(0)->members.at(0).rot);

  pos.Set(0, 9, 0);
  gameState->MoveAgent(agent, pos);
  gameState->MoveAgentToSide(agent);
  pos.Set(0, -SoccerField::HalfFieldHeight, GameState::beamHeight);
  ASSERT_EQ(pos, agent.pos);

  pos.Set(0, -9, 0);
  gameState->MoveAgent(agent, pos);
  gameState->MoveAgentToSide(agent);
  pos.Set(0, SoccerField::HalfFieldHeight, GameState::beamHeight);
  ASSERT_EQ(pos, agent.pos);

  pos.Set(-5, -6, GameState::beamHeight);
  rot.Euler(1.0, 0.5, 0.7);
  gameState->MoveAgent(agent, pos, rot);
  ASSERT_EQ(agent.pos, pos);
  ASSERT_EQ(rot, agent.rot);
  ASSERT_EQ(pos, gameState->teams.at(0)->members.at(0).pos);
  ASSERT_EQ(rot, gameState->teams.at(0)->members.at(0).rot);

  pos.Set(7, 8, GameState::beamHeight);
  rot.Euler(0, 0, 1.25);
  for (int i = 0; i < 100; i++) {
    bool result = gameState->beamAgent(1, "blue", 7, 8, 1.25);
    ASSERT_TRUE(result);
    ASSERT_LE(agent.pos.Distance(pos), 0.15);
    ASSERT_LE(fabs(agent.rot.Euler().Z() - rot.Euler().Z()), 0.1);
  }
}

class GameStateTest_fullTeams : public GameStateTest_basic {
  protected:
  virtual void SetUp() {
    GameStateTest_basic::SetUp();
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 11; j++) {
        gameState->addAgent(j + 1, teamNames[i]);
      }
    }
  }

  virtual void resetPositions() {
    for (int i = 0; i < 2; i++) {
      for (int j = 0; j < 11; j++) {
        GameState::Agent &agent = gameState->teams.at(0)->members.at(j);
        gameState->MoveAgent(agent, math::Vector3<double>
                             (0, 0, GameState::beamHeight));
      }
    }
  }

  virtual void resetPositions(int team) {
    if (team != 0 || team != 1) {
      return;
    }
    for (int j = 0; j < 11; j++) {
      GameState::Agent &agent = gameState->teams.at(team)->members.at(j);
      gameState->MoveAgent(agent, math::Vector3<double>
                           (0, 0, GameState::beamHeight));
    }
  }
};

/// \brief Test for whether beforeKickOff play mode transitions correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_beforeKickOff_kickOff) {
  // try first half
  ASSERT_EQ(gameState->GetHalf(), GameState::FIRST_HALF);
  while (gameState->getGameTime() < GameState::SecondsBeforeKickOff + 1) {
    gameState->Update();
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ(gameState->GetBall(), SoccerField::BallCenterPosition);
      ASSERT_EQ(gameState->getCurrentState()->name, "BeforeKickOff");
    }
    else
    {
      ASSERT_EQ(gameState->getCurrentState()->name, "KickOff_Left");
    }
  }

  // try second half
  gameState->SetHalf(GameState::SECOND_HALF);
  ASSERT_EQ(gameState->GetHalf(), GameState::SECOND_HALF);
  while (gameState->getGameTime() < GameState::SecondsBeforeKickOff + 1) {
    gameState->Update();
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ(gameState->GetBall(), SoccerField::BallCenterPosition);
      ASSERT_EQ(gameState->getCurrentState()->name, "BeforeKickOff");
    }
    else
    {
      ASSERT_EQ(gameState->getCurrentState()->name, "KickOff_Right");
    }
  }
}

/// \brief Test for whether KickOff play mode transitions correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_kickOff_playOn) {
  vector<State *> states;
  states.push_back(gameState->kickOffLeftState.get());
  states.push_back(gameState->kickOffRightState.get());

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  // Test for both left && right kick offs
  for (size_t i = 0; i < states.size(); i++) {
    State *state = states.at(i);

    // test for transition when KickOff times out after 15 secs
    gameState->setCycleCounter(0);
    gameState->SetCurrent(state);
    while (gameState->getGameTime() < GameState::SecondsKickOff + 1) {
      gameState->Update();
      // cout << gameState->getGameTime() << " " << gameState->
      // getCurrentState()->name << " " << gameState->getCurrentState()->
      // getElapsedTime() << endl;
      if (gameState->getGameTime() < GameState::SecondsKickOff) {
        ASSERT_EQ(gameState->getCurrentState()->name, state->name);
      }
      else
      {
        ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
      }
    }

    // test for transition when ball is touched
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact(
      new GameState::BallContact(1, gameState->teams.at(i)->side,
                                 gameState->getGameTime(),
                                 math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_TRUE(gameState->touchBallKickoff != NULL);
    ASSERT_EQ(gameState->touchBallKickoff, ballContact.get());
    ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");

    // test for transition when double touching occurs
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact2(
      new GameState::BallContact(1, gameState->teams.at(i)->side,
                                 gameState->getGameTime(),
                                 math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact2);
    for (int j = 0; j < 10; j++) {
      gameState->Update();
      ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
    }
    boost::shared_ptr<GameState::BallContact> ballContact3(
      new GameState::BallContact(1, gameState->teams.at(i)->side,
                                 gameState->getGameTime(),
                                 math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact3);
    gameState->Update();
    ASSERT_EQ(gameState->getCurrentState()->name, states.at((i + 1) % 2)->name);

    // test that transition to other kickoff does !happen when
    // double touch does !occur
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact4(
      new GameState::BallContact(1, gameState->teams.at(i)->side,
                                 gameState->getGameTime(),
                                 math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact4);
    for (int j = 0; j < 10; j++) {
      gameState->Update();
      if (j == 5) {
        boost::shared_ptr<GameState::BallContact> ballContact5(
          new GameState::BallContact(5,
                                     gameState->teams.at((i + 1) % 2)->side,
                                     gameState->getGameTime(),
                                     math::Vector3<double>(0, 0, 0)));
        gameState->ballContactHistory.push_back(ballContact5);
      }
    }
    boost::shared_ptr<GameState::BallContact> ballContact6(
      new GameState::BallContact(1, gameState->teams.at(i)->side,
                                 gameState->getGameTime(),
                                 math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact6);
    gameState->Update();
    ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
  }
}

/// \brief Test for whether PlayOn play mode transitions to kickIn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_kickIn) {
  vector<State *> states;
  states.push_back(gameState->kickInLeftState.get());
  states.push_back(gameState->kickInRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(0, -15,
                          SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(0, 15,
                          SoccerField::BallRadius));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    for (size_t j = 0; j < ballPositions.size(); j++) {
      gameState->MoveBall(math::Vector3<double>(0, 0, 0));
      gameState->SetCurrent(gameState->playState.get());
      gameState->Update();
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

      ballContact = boost::shared_ptr<GameState::BallContact>(
                      new GameState::BallContact(1,
                          gameState->teams.at(i)->side,
                          gameState->getGameTime(),
                          math::Vector3<double>(0, 0, 0)));
      gameState->ballContactHistory.push_back(ballContact);

      gameState->MoveBall(ballPositions.at(j));
      gameState->Update();
      ASSERT_EQ(states.at((i + 1) % 2)->name,
                gameState->getCurrentState()->name);
    }
  }
}

/// \brief Test for whether PlayOn play mode transitions to cornerKick correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_cornerKick) {
  vector<State *> states;
  states.push_back(gameState->cornerKickLeftState.get());
  states.push_back(gameState->cornerKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 1),
                            5, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 1,
                            -5, SoccerField::BallRadius));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    gameState->MoveBall(math::Vector3<double>(0, 0, 0));
    gameState->SetCurrent(gameState->playState.get());
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->MoveBall(ballPositions.at(i));
    // cout << gameState->getLastBallContact()->side << " " <<
    // gameState->GetBall() << " " << gameState->teams.at(i)->side << endl;
    gameState->Update();
    ASSERT_EQ(states.at((i + 1) % 2)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether PlayOn play mode transitions to goal correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_goal) {
  vector<State *> states;
  states.push_back(gameState->goalLeftState.get());
  states.push_back(gameState->goalRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 0.5),
                            1, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 0.5,
                            -1, SoccerField::BallRadius));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    gameState->MoveBall(math::Vector3<double>(0, 0, 0));
    gameState->SetCurrent(gameState->playState.get());
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    gameState->MoveBall(ballPositions.at(i));
    gameState->Update();
    ASSERT_EQ(states.at((i + 1) % 2)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether PlayOn play mode transitions to goalKick correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_playOn_goalKick) {
  vector<State *> states;
  states.push_back(gameState->goalKickLeftState.get());
  states.push_back(gameState->goalKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(
                            -(SoccerField::HalfFieldWidth + 1),
                            5, SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(
                            SoccerField::HalfFieldWidth + 1,
                            -5, SoccerField::BallRadius));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    gameState->MoveBall(math::Vector3<double>(0, 0, 0));
    gameState->SetCurrent(gameState->playState.get());
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1,
                        gameState->teams.at((i + 1) % 2)->side,
                        gameState->getGameTime(),
                        math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->MoveBall(ballPositions.at(i));
    gameState->Update();
    ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether goal play mode transitions to kickOff correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_goal_kickOff) {
  vector<State *> beforeStates;
  beforeStates.push_back(gameState->goalLeftState.get());
  beforeStates.push_back(gameState->goalRightState.get());

  vector<State *> afterStates;
  afterStates.push_back(gameState->kickOffRightState.get());
  afterStates.push_back(gameState->kickOffLeftState.get());

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);

  for (size_t i = 0; i < beforeStates.size(); i++) {
    // check that valid goal transitions correctly
    gameState->setCycleCounter(0);
    gameState->SetCurrent(beforeStates.at(i));
    gameState->teams.at(i)->canScore = true;

    while (gameState->getGameTime() < GameState::SecondsGoalPause) {
      ASSERT_EQ(beforeStates.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ(afterStates.at(i)->name, gameState->getCurrentState()->name);

    gameState->SetCurrent(beforeStates.at(i));
    gameState->teams.at(i)->canScore = false;
    gameState->Update();
    ASSERT_EQ(afterStates.at(i)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether kickIn play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_kickIn_playOn) {
  vector<State *> states;
  states.push_back(gameState->kickInLeftState.get());
  states.push_back(gameState->kickInRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    // transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    // transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether cornerKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_cornerKick_playOn) {
  vector<State *> states;
  states.push_back(gameState->cornerKickLeftState.get());
  states.push_back(gameState->cornerKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    // transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    // transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether freeKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_freeKick_playOn) {
  vector<State *> states;
  states.push_back(gameState->freeKickLeftState.get());
  states.push_back(gameState->freeKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    // transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    // transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether goalKick play mode transitions to playOn correctly
TEST_F(GameStateTest_fullTeams, GameState_transition_goalKick_playOn) {
  vector<State *> states;
  states.push_back(gameState->goalKickLeftState.get());
  states.push_back(gameState->goalKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(-16, 5,
                          SoccerField::BallRadius));
  ballPositions.push_back(math::Vector3<double>(16, -5,
                          SoccerField::BallRadius));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    // transition from timing out
    gameState->setCycleCounter(0);
    gameState->MoveBall(ballPositions.at(i));
    gameState->SetCurrent(states.at(i));

    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    // transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    // move ball out of penalty area
    gameState->MoveBallToCenter();
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether when running full game, game transitions to
/// kickOffRight at end of first half && to gameOver at the end of the
/// second half
TEST_F(GameStateTest_fullTeams, GameState_transition_checkTiming) {
  double firstHalfTime = GameState::SecondsEachHalf +
                         GameState::SecondsBeforeKickOff;
  double firstHalfKickOffTime = GameState::SecondsBeforeKickOff +
                                GameState::SecondsKickOff;
  double secondHalfTime = GameState::SecondsBeforeKickOff +
                          GameState::SecondsFullGame;
  double secondHalfKickOffTime = GameState::SecondsKickOff + firstHalfTime;

  while (gameState->getGameTime() < firstHalfTime) {
    ASSERT_TRUE(gameState->getElapsedGameTime() < GameState::SecondsEachHalf);
    ASSERT_TRUE(gameState->GetHalf() == GameState::FIRST_HALF);
    ASSERT_TRUE(gameState->teams.at(0)->side == GameState::Team::LEFT
                && gameState->teams.at(1)->side == GameState::Team::RIGHT);
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ("BeforeKickOff", gameState->getCurrentState()->name);
    }
    else if (gameState->getGameTime() < firstHalfKickOffTime)
    {
      ASSERT_EQ("KickOff_Left", gameState->getCurrentState()->name);
    }
else
{
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
    }
    gameState->Update();
  }
  ASSERT_EQ("KickOff_Right", gameState->getCurrentState()->name);
  while (gameState->getGameTime() < secondHalfTime) {
    ASSERT_TRUE(gameState->GetHalf() == GameState::SECOND_HALF);
    ASSERT_TRUE(gameState->teams.at(0)->side == GameState::Team::RIGHT
                && gameState->teams.at(1)->side == GameState::Team::LEFT);
    ASSERT_TRUE(gameState->getElapsedGameTime() < GameState::SecondsEachHalf);
    if (gameState->getGameTime() < secondHalfKickOffTime) {
      ASSERT_EQ("KickOff_Right", gameState->getCurrentState()->name);
    }
else
{
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
    }
    gameState->Update();
  }
  ASSERT_EQ("GameOver", gameState->getCurrentState()->name);
}

/// \brief Test to check whether the GameState CheckCanScore function
/// works as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckCanScore) {
  vector<State *> states;
  states.push_back(gameState->kickOffLeftState.get());
  states.push_back(gameState->kickOffRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  math::Vector3<double> pos;

  for (size_t i = 0; i < states.size(); i++) {
    // case 1: kickoff agent touches ball, teammate touches ball outside circle
    // team should be able to score
    gameState->SetCurrent(states.at(i));
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    pos.Set(5, 5, SoccerField::BallRadius);
    ballContact = boost::shared_ptr<GameState::BallContact>(new
                  GameState::BallContact(2, gameState->teams.at(i)->side,
                                         gameState->getGameTime(), pos));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_TRUE(gameState->teams.at(i)->canScore);

    // case 2: kickoff agent touches ball, teammate touches ball inside circle
    // team should !be able to score
    gameState->SetCurrent(states.at(i));
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(2, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);

    // case 3: kickoff agent touches ball, && touches it again afterwards
    // team should !be able to score
    gameState->SetCurrent(states.at(i));
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);

    // case 4: kickoff agent touches ball, &&
    // someone on opposing team touches it
    // team should be able to score
    gameState->SetCurrent(states.at(i));
    gameState->Update();
    ASSERT_FALSE(gameState->teams.at(i)->canScore);
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1, gameState->teams.at(i)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ballContact = boost::shared_ptr<GameState::BallContact>(
                    new GameState::BallContact(1,
                        gameState->teams.at((i + 1) % 2)->side,
                        gameState->getGameTime(),
                        gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_TRUE(gameState->teams.at(i)->canScore);
  }
}

/// \brief Test to check whether the CheckIllegalDefense
/// function works as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckIllegalDefense) {
  for (int i = 0; i < 2; i++) {
    math::Box penaltyBox;
    math::Vector3<double> goalCenter;
    if (i == 0) {
      penaltyBox = SoccerField::PenaltyBoxLeft;
      goalCenter = SoccerField::GoalCenterLeft;
    }
else
{
      penaltyBox = SoccerField::PenaltyBoxRight;
      goalCenter = SoccerField::GoalCenterRight;
    }
    math::Vector3<double> penaltyPos = penaltyBox.Center();
    gameState->SetCurrent(gameState->playState.get());

    // test whether fourth agent in penalty box gets beamed out
    resetPositions();
    gameState->Update();
    for (int j = 0; j < 3; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j + 1);
      gameState->MoveAgent(agent, penaltyPos);
    }
    gameState->Update();
    for (int j = 0; j < 3; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j + 1);
      ASSERT_TRUE(penaltyBox.Contains(agent.pos));
    }
    GameState::Agent &agent2 = gameState->teams.at(i)->members.at(4);
    gameState->MoveAgent(agent2, penaltyPos);
    gameState->Update();
    ASSERT_FALSE(penaltyBox.Contains(agent2.pos));

    // test whether goalie (if fourth goalie) stays in penalty box && farthest
    // agent gets beamed out
    resetPositions();
    gameState->Update();
    for (int j = 1; j < 4; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j);
      if (j == 3) {
        // this agent is farthest away from goal
        gameState->MoveAgent(agent, penaltyPos);
      }
else
{
        gameState->MoveAgent(agent, goalCenter);
      }
    }
    gameState->Update();
    GameState::Agent &agent3 = gameState->teams.at(i)->members.at(0);
    gameState->MoveAgent(agent3, penaltyPos);
    gameState->Update();
    ASSERT_TRUE(penaltyBox.Contains(agent3.pos));
    for (int j = 1; j < 4; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j);
      if (j == 3) {
        // this agent is farthest away from goal
        ASSERT_FALSE(penaltyBox.Contains(agent.pos));
      }
else
{
        ASSERT_TRUE(penaltyBox.Contains(agent.pos));
      }
    }

    // test that nothing happens if opponent also goes into goal box
    resetPositions();
    gameState->Update();
    for (int j = 0; j < 3; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j);
      gameState->MoveAgent(agent, penaltyPos);
    }
    gameState->Update();
    GameState::Agent &agent4 = gameState->teams.at((i + 1) % 2)->members.at(1);
    gameState->MoveAgent(agent4, penaltyPos);
    gameState->Update();
    for (int j = 0; j < 3; j++) {
      GameState::Agent &agent = gameState->teams.at(i)->members.at(j);
      ASSERT_TRUE(penaltyBox.Contains(agent.pos));
    }
  }
}

/// \brief Test to check whether the CheckCrowding function
/// in gameState is working as intended
TEST_F(GameStateTest_fullTeams, GameState_CheckCrowding) {
  math::Vector3<double> testBallPos(5, 0, SoccerField::BallRadius);
  math::Vector3<double> crowdingEnablePos(5 + 0.5 *
                                          GameState::crowdingEnableDist, 0,
                                          SoccerField::BallRadius);
  math::Vector3<double> innerRadius(5 + 0.5 * GameState::crowdingReposDist2,
                                    0, SoccerField::BallRadius);
  math::Vector3<double> innerRadius2(5 + 0.75 *
                                     GameState::crowdingReposDist2, 0,
                                     SoccerField::BallRadius);
  math::Vector3<double> outerRadius(5 + 0.5 * GameState::crowdingReposDist3,
                                    0, SoccerField::BallRadius);
  math::Vector3<double> outerRadius2(5 + 0.75 *
                                     GameState::crowdingReposDist3, 0,
                                     SoccerField::BallRadius);

  gameState->SetCurrent(gameState->playState.get());
  gameState->MoveBall(testBallPos);
  // move enemy agent close to ball to enable crowding

  for (int i = 0; i < 2; i++) {
    resetPositions();
    // put agent on other team within crowding radius to ensure that crowding
    // rules are enabled
    GameState::Agent &agent = gameState->teams.at((i + 1) % 2)->members.at(1);
    gameState->MoveAgent(agent, crowdingEnablePos);
    gameState->Update();
    ASSERT_EQ(agent.pos, crowdingEnablePos);

    // test for whether inner radius crowding check works
    GameState::Agent &agent2 = gameState->teams.at(i)->members.at(1);
    gameState->MoveAgent(agent2, innerRadius);
    gameState->Update();
    ASSERT_EQ(agent2.pos, innerRadius);
    GameState::Agent &agent3 = gameState->teams.at(i)->members.at(2);
    gameState->MoveAgent(agent3, innerRadius2);
    gameState->Update();
    ASSERT_GE(agent3.pos.Distance(gameState->GetBall()),
              GameState::crowdingReposDist2);

    // test for whether the outer radius crowding check works
    GameState::Agent &agent4 = gameState->teams.at(i)->members.at(3);
    gameState->MoveAgent(agent4, outerRadius);
    gameState->Update();
    ASSERT_EQ(agent4.pos, outerRadius);
    GameState::Agent &agent5 = gameState->teams.at(i)->members.at(4);
    gameState->MoveAgent(agent5, outerRadius2);
    gameState->Update();
    ASSERT_GE(agent5.pos.Distance(gameState->GetBall()),
              GameState::crowdingReposDist3);
  }
}

/// \brief Test to check whether the CheckImmobility function in gameState is
/// working as intended
TEST_F(GameStateTest_basic, GameState_CheckImmobilityFallen) {
  math::Vector3<double> pos(0, 0, 0.4);
  vector<math::Vector3<double> >fallenPos;
  fallenPos.push_back(math::Vector3<double>(0.0, 0.0, 0.1));
  fallenPos.push_back(math::Vector3<double>(0.1, 0.1, 0.1));
  gameState->addAgent(1, "blue");
  gameState->SetCurrent(gameState->playState.get());

  // check immobility && fallen for goalie
  GameState::Agent &agent = gameState->teams.at(0)->members.at(0);
  while (gameState->getGameTime() < 2 * GameState::immobilityTimeLimit) {
    ASSERT_EQ(agent.pos, pos);
    gameState->Update();
  }
  ASSERT_NE(agent.pos, pos);

  gameState->setCycleCounter(0);
  int c = -1;
  while (gameState->getGameTime() < 2 * GameState::fallenTimeLimit) {
    c++;
    gameState->MoveAgent(agent, fallenPos.at(c % 2));
    ASSERT_EQ(agent.pos, fallenPos.at(c % 2));
    gameState->Update();
  }
  ASSERT_NE(agent.pos, fallenPos.at(c % 2));

  // check immobility && fallen for non-goalie
  gameState->setCycleCounter(0);
  gameState->addAgent(2, "blue");
  GameState::Agent &agent2 = gameState->teams.at(0)->members.at(1);
  while (gameState->getGameTime() < GameState::immobilityTimeLimit) {
    ASSERT_EQ(agent2.pos, pos);
    gameState->Update();
  }
  ASSERT_NE(agent2.pos, pos);

  gameState->setCycleCounter(0);
  c = -1;
  while (gameState->getGameTime() < GameState::fallenTimeLimit) {
    c++;
    gameState->MoveAgent(agent2, fallenPos.at(c % 2));
    ASSERT_EQ(agent2.pos, fallenPos.at(c % 2));
    gameState->Update();
  }
  ASSERT_NE(agent2.pos, fallenPos.at(c % 2));
}

TEST_F(GameStateTest_fullTeams, GameState_DropBall) {
  gameState->MoveBallToCenter();

  for (int i = 0; i < 2; i++) {
    resetPositions();
    GameState::Team *allowedTeam = gameState->teams.at(i);
    GameState::Team *notAllowedTeam = gameState->teams.at((i + 1) % 2);
    gameState->DropBallImpl(allowedTeam->side);

    for (int j = 0; j < 11; j++) {
      GameState::Agent &agent = notAllowedTeam->members.at(j);
      ASSERT_GE(agent.pos.Distance(gameState->GetBall()),
                GameState::dropBallRadius);
    }
  }
}

int main(int argc, char **argv) {
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


