/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <string>
#include <boost/scoped_ptr.hpp>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/KickOffRightState.hh"

using namespace ignition;

/////////////////////////////////////////////////
KickOffRightState::KickOffRightState(const std::string &_name,
                                     GameState *_gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void KickOffRightState::Initialize()
{
  gameState->touchBallKickoff = NULL;
  gameState->ballContactHistory.clear();
  // boost::shared_ptr<GameState::BallContact> initContact(new GameState::BallContact(-1, GameState::Team::RIGHT, gameState->getGameTime(), SoccerField::CenterOfField));
  // gameState->ballContactHistory.push_back(initContact);
  for (size_t i = 0; i < gameState->teams.size(); i++) {
    GameState::Team *team = gameState->teams.at(i);
    team->canScore = false;
  }
  gameState->MoveBallToCenter();
  gameState->ReleasePlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void KickOffRightState::Update()
{
  if (not hasInitialized) {
    Initialize();
  }

  //check for agents that violate sides
  for (size_t i = 0; i < gameState->teams.size(); ++i) {
    GameState::Team *currTeam = gameState->teams.at(i);
    for (size_t j = 0; j < currTeam->members.size(); ++j) {
      GameState::Agent &agent = currTeam->members.at(j);
      math::Vector3<double> agentPosNoZ(agent.pos.X(), agent.pos.Y(), 0);
      //if on kicking team, must stay in circle and own side.
      if (currTeam->side == GameState::Team::RIGHT and (agent.pos.X() > 0 and agentPosNoZ.Distance(SoccerField::CenterOfField) > SoccerField::CenterCircleRadius)) {
        // move them to side of field for now
        gameState->MoveAgentToSide(agent);

        //if on defending team, cannot cross line or go inside circle.
      } else if (currTeam->side == GameState::Team::LEFT and (agent.pos.X() < 0 or agentPosNoZ.Distance(SoccerField::CenterOfField) < SoccerField::CenterCircleRadius)) {
        // move them to side of field for now
        gameState->MoveAgentToSide(agent);
      }
    }
  }

  State::Update();

  // After some time, go to play mode.
  if (getElapsedTime() >= GameState::SecondsKickOff) {
    gameState->DropBallImpl(GameState::Team::NEITHER);
    gameState->SetCurrent(gameState->playState.get());
  } else if (hasBallContactOccurred()) {
    gameState->touchBallKickoff = gameState->getLastBallContact();
    gameState->SetCurrent(gameState->playState.get());
  }
}
