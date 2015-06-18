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

#include "ignition/math.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "gtest/gtest.h"
#include <boost/scoped_ptr.hpp>

using namespace std;
using namespace ignition;

class GameStateTest_basic : public ::testing::Test
{
  protected:
    GameState *gameState;
    string teamNames[3] = {"blue", "red", "green"}; //green is third team and shouldnt be added

    virtual void SetUp()
    {
      gameState = new GameState();
    }

    virtual void TearDown()
    {
      delete gameState;
    }

};

/// \brief Test whether GameState constructor and destructor works
TEST_F(GameStateTest_basic, GameState_construct_delete)
{
  SUCCEED();
}

/// \brief Test for adding teams and agents
TEST_F(GameStateTest_basic, GameState_add_teams_agents)
{
  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 15; j++) {
      bool returnValue = gameState->addAgent(j + 1, teamNames[i]);
      if (i >= 2 or j + 1 >= 12) {
        ASSERT_FALSE(returnValue);
      } else {
        ASSERT_TRUE(returnValue);
      }
    }
  }

  for (int i = 0; i < 2; i++) {
    for (int j = 0; j < 2; j++) {
      ASSERT_FALSE(gameState->addAgent(j + 1, teamNames[i]));
    }
  }

  ASSERT_EQ(gameState->teams.size(), 2u);
  for (int i = 0; i < 2; i++) {
    ASSERT_EQ(gameState->teams.at(i)->members.size(), 11u);
    if (i == 0) {
      ASSERT_EQ(gameState->teams.at(i)->name, "blue");
      ASSERT_EQ(gameState->teams.at(i)->side, GameState::Team::LEFT);
    } else {
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

/// \brief Test for removing teams and agents
TEST_F(GameStateTest_basic, GameState_remove_agents)
{
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
TEST_F(GameStateTest_basic, GameState_add_agent_0)
{
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

class GameStateTest_playmode : public GameStateTest_basic
{
  protected:
    virtual void SetUp()
    {
      GameStateTest_basic::SetUp();
      for (int i = 0; i < 2; i++) {
        for (int j = 0; j < 11; j++) {
          gameState->addAgent(j + 1, teamNames[i]);
        }
      }
    }
};

/// \brief Test for whether beforeKickOff play mode transitions correctly
TEST_F(GameStateTest_playmode, GameState_transition_beforeKickOff_kickOff)
{
  //try first half
  ASSERT_EQ(gameState->GetHalf(), GameState::FIRST_HALF);
  while (gameState->getGameTime() < 6) {
    gameState->Update();
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ(gameState->GetBall(), SoccerField::BallCenterPosition);
      ASSERT_EQ(gameState->getCurrentState()->name, "BeforeKickOff");
    } else {
      ASSERT_EQ(gameState->getCurrentState()->name, "KickOff_Left");
    }
  }

  // try second half
  gameState->SetHalf(GameState::SECOND_HALF);
  ASSERT_EQ(gameState->GetHalf(), GameState::SECOND_HALF);
  while (gameState->getGameTime() < 6) {
    gameState->Update();
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ(gameState->GetBall(), SoccerField::BallCenterPosition);
      ASSERT_EQ(gameState->getCurrentState()->name, "BeforeKickOff");
    } else {
      ASSERT_EQ(gameState->getCurrentState()->name, "KickOff_Right");
    }
  }
}

/// \brief Test for whether KickOff play mode transitions correctly
TEST_F(GameStateTest_playmode, GameState_transition_kickOff_playOn)
{
  vector<State *> states;
  states.push_back(gameState->kickOffLeftState.get());
  states.push_back(gameState->kickOffRightState.get());

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  //Test for both left and right kick offs
  for (size_t i = 0; i < states.size(); i++) {
    State *state = states.at(i);

    //test for transition when KickOff times out after 15 secs
    gameState->setCycleCounter(0);
    gameState->SetCurrent(state);
    while (gameState->getGameTime() < 16) {
      gameState->Update();
      // cout << gameState->getGameTime() << " " << gameState->getCurrentState()->name << " " << gameState->getCurrentState()->getElapsedTime() << endl;
      if (gameState->getGameTime() < GameState::SecondsKickOff) {
        ASSERT_EQ(gameState->getCurrentState()->name, state->name);
      } else {
        ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
      }
    }

    //test for transition when ball is touched
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_TRUE(gameState->touchBallKickoff != NULL);
    ASSERT_EQ(gameState->touchBallKickoff, ballContact.get());
    ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");

    //test for transition when double touching occurs
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact2(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact2);
    for (int j = 0; j < 10; j++) {
      gameState->Update();
      ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
    }
    boost::shared_ptr<GameState::BallContact> ballContact3(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact3);
    gameState->Update();
    ASSERT_EQ(gameState->getCurrentState()->name, states.at((i + 1) % 2)->name);

    //test that transition to other kickoff does not happen when double touch does not occur
    gameState->SetCurrent(state);
    gameState->Update();
    boost::shared_ptr<GameState::BallContact> ballContact4(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact4);
    for (int j = 0; j < 10; j++) {
      gameState->Update();
      if (j == 5) {
        boost::shared_ptr<GameState::BallContact> ballContact5(new GameState::BallContact(5, gameState->teams.at((i + 1) % 2)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
        gameState->ballContactHistory.push_back(ballContact5);
      }
    }
    boost::shared_ptr<GameState::BallContact> ballContact6(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact6);
    gameState->Update();
    ASSERT_EQ(gameState->getCurrentState()->name, "PlayOn");
  }
}

/// \brief Test for whether PlayOn play mode transitions to kickIn correctly
TEST_F(GameStateTest_playmode, GameState_transition_playOn_kickIn)
{
  vector<State *> states;
  states.push_back(gameState->kickInLeftState.get());
  states.push_back(gameState->kickInRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(0, -15, 0.042));
  ballPositions.push_back(math::Vector3<double>(0, 15, 0.042));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    for (size_t j = 0; j < ballPositions.size(); j++) {
      gameState->MoveBall(math::Vector3<double>(0, 0, 0));
      gameState->SetCurrent(gameState->playState.get());
      gameState->Update();
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

      ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
      gameState->ballContactHistory.push_back(ballContact);

      gameState->MoveBall(ballPositions.at(j));
      gameState->Update();
      ASSERT_EQ(states.at((i + 1) % 2)->name, gameState->getCurrentState()->name);
    }
  }
}

/// \brief Test for whether PlayOn play mode transitions to cornerKick correctly
TEST_F(GameStateTest_playmode, GameState_transition_playOn_cornerKick)
{
  vector<State *> states;
  states.push_back(gameState->cornerKickLeftState.get());
  states.push_back(gameState->cornerKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(-16, 5, 0.042));
  ballPositions.push_back(math::Vector3<double>(16, -5, 0.042));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    gameState->MoveBall(math::Vector3<double>(0, 0, 0));
    gameState->SetCurrent(gameState->playState.get());
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->MoveBall(ballPositions.at(i));
    // cout << gameState->getLastBallContact()->side << " " << gameState->GetBall() << " " << gameState->teams.at(i)->side << endl;
    gameState->Update();
    ASSERT_EQ(states.at((i + 1) % 2)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether PlayOn play mode transitions to goal correctly
TEST_F(GameStateTest_playmode, GameState_transition_playOn_goal)
{
  vector<State *> states;
  states.push_back(gameState->goalLeftState.get());
  states.push_back(gameState->goalRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(-15.5, 1, 0.042));
  ballPositions.push_back(math::Vector3<double>(15.5, -1, 0.042));

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
TEST_F(GameStateTest_playmode, GameState_transition_playOn_goalKick)
{
  vector<State *> states;
  states.push_back(gameState->goalKickLeftState.get());
  states.push_back(gameState->goalKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(-16, 5, 0.042));
  ballPositions.push_back(math::Vector3<double>(16, -5, 0.042));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {
    gameState->MoveBall(math::Vector3<double>(0, 0, 0));
    gameState->SetCurrent(gameState->playState.get());
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at((i + 1) % 2)->side, gameState->getGameTime(), math::Vector3<double>(0, 0, 0)));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->MoveBall(ballPositions.at(i));
    // cout << gameState->getLastBallContact()->side << " " << gameState->GetBall() << " " << gameState->teams.at(i)->side << endl;
    gameState->Update();
    ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether goal play mode transitions to kickOff correctly
TEST_F(GameStateTest_playmode, GameState_transition_goal_kickOff)
{
  vector<State *> beforeStates;
  beforeStates.push_back(gameState->goalLeftState.get());
  beforeStates.push_back(gameState->goalRightState.get());

  vector<State *> afterStates;
  afterStates.push_back(gameState->kickOffRightState.get());
  afterStates.push_back(gameState->kickOffLeftState.get());

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);

  for (size_t i = 0; i < beforeStates.size(); i++) {
    ///check that valid goal transitions correctly
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
TEST_F(GameStateTest_playmode, GameState_transition_kickIn_playOn)
{
  vector<State *> states;
  states.push_back(gameState->kickInLeftState.get());
  states.push_back(gameState->kickInRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {

    //transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    //transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether cornerKick play mode transitions to playOn correctly
TEST_F(GameStateTest_playmode, GameState_transition_cornerKick_playOn)
{
  vector<State *> states;
  states.push_back(gameState->cornerKickLeftState.get());
  states.push_back(gameState->cornerKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {

    //transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    //transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether freeKick play mode transitions to playOn correctly
TEST_F(GameStateTest_playmode, GameState_transition_freeKick_playOn)
{
  vector<State *> states;
  states.push_back(gameState->freeKickLeftState.get());
  states.push_back(gameState->freeKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {

    //transition from timing out
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    //transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    ballContact = boost::shared_ptr<GameState::BallContact>(new GameState::BallContact(1, gameState->teams.at(i)->side, gameState->getGameTime(), gameState->GetBall()));
    gameState->ballContactHistory.push_back(ballContact);
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether goalKick play mode transitions to playOn correctly
TEST_F(GameStateTest_playmode, GameState_transition_goalKick_playOn)
{
  vector<State *> states;
  states.push_back(gameState->goalKickLeftState.get());
  states.push_back(gameState->goalKickRightState.get());
  boost::shared_ptr<GameState::BallContact> ballContact;
  vector<math::Vector3<double> > ballPositions;
  ballPositions.push_back(math::Vector3<double>(-16, 5, 0.042));
  ballPositions.push_back(math::Vector3<double>(16, -5, 0.042));

  ASSERT_EQ(GameState::Team::LEFT, gameState->teams.at(0)->side);
  ASSERT_EQ(GameState::Team::RIGHT, gameState->teams.at(1)->side);
  for (size_t i = 0; i < states.size(); i++) {

    //transition from timing out
    gameState->setCycleCounter(0);
    gameState->MoveBall(ballPositions.at(i));
    gameState->SetCurrent(states.at(i));

    while (gameState->getGameTime() < GameState::SecondsKickIn) {
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
      gameState->Update();
    }
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);

    //transition to play on from touching the ball
    gameState->setCycleCounter(0);
    gameState->SetCurrent(states.at(i));
    while (gameState->getGameTime() < GameState::SecondsKickInPause) {
      gameState->Update();
      ASSERT_EQ(states.at(i)->name, gameState->getCurrentState()->name);
    }
    //move ball out of penalty area
    gameState->MoveBallToCenter();
    gameState->Update();
    ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
  }
}

/// \brief Test for whether game transitions to kickOffRight at end of first half and to gameOver at the end of the second half
TEST_F(GameStateTest_playmode, GameState_transition_checkTiming)
{
  double firstHalfTime = GameState::SecondsEachHalf +  GameState::SecondsBeforeKickOff;
  double firstHalfKickOffTime = GameState::SecondsBeforeKickOff + GameState::SecondsKickOff;
  double secondHalfTime = GameState::SecondsBeforeKickOff + GameState::SecondsFullGame;
  double secondHalfKickOffTime = GameState::SecondsKickOff + firstHalfTime;

  while (gameState->getGameTime() < firstHalfTime) {
    // cout << gameState->getGameTime() << " " << gameState->getCurrentState()->name << " " << gameState->getCurrentState()->getElapsedTime() << endl;
    ASSERT_TRUE(gameState->getElapsedGameTime() < GameState::SecondsEachHalf);
    ASSERT_TRUE(gameState->GetHalf() == GameState::FIRST_HALF);
    ASSERT_TRUE(gameState->teams.at(0)->side == GameState::Team::LEFT and gameState->teams.at(1)->side == GameState::Team::RIGHT);
    if (gameState->getGameTime() < GameState::SecondsBeforeKickOff) {
      ASSERT_EQ("BeforeKickOff", gameState->getCurrentState()->name);
    } else if (gameState->getGameTime() < firstHalfKickOffTime) {
      ASSERT_EQ("KickOff_Left", gameState->getCurrentState()->name);
    } else {
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
    }
    gameState->Update();
  }
  ASSERT_EQ("KickOff_Right", gameState->getCurrentState()->name);
  while (gameState->getGameTime() < secondHalfTime) {
    ASSERT_TRUE(gameState->GetHalf() == GameState::SECOND_HALF);
    ASSERT_TRUE(gameState->teams.at(0)->side == GameState::Team::RIGHT and gameState->teams.at(1)->side == GameState::Team::LEFT);
    ASSERT_TRUE(gameState->getElapsedGameTime() < GameState::SecondsEachHalf);
    if (gameState->getGameTime() < secondHalfKickOffTime) {
      ASSERT_EQ("KickOff_Right", gameState->getCurrentState()->name);
    } else {
      ASSERT_EQ("PlayOn", gameState->getCurrentState()->name);
    }
    gameState->Update();
  }
  ASSERT_EQ("GameOver", gameState->getCurrentState()->name);
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}


