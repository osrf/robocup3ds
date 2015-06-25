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

#include <string>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/KickOffLeftState.hh"
#include "robocup3ds/states/PlayOnState.hh"

/////////////////////////////////////////////////
KickOffLeftState::KickOffLeftState(const std::string &_name,
                                   GameState *const _gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void KickOffLeftState::Initialize()
{
  this->gameState->touchBallKickoff = NULL;
  this->gameState->ballContactHistory.clear();
  for (size_t i = 0; i < this->gameState->teams.size(); ++i)
  {
    std::shared_ptr<GameState::Team> team = this->gameState->teams.at(i);
    team->canScore = false;
  }
  this->gameState->MoveBallToCenter();
  this->gameState->ReleasePlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void KickOffLeftState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }

  // check for agents that violate sides
  gameState->CheckOffSidesOnKickOff(GameState::Team::Side::LEFT);
  State::Update();

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsKickOff)
  {
    this->gameState->DropBallImpl(GameState::Team::Side::NEITHER);
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
  else if (this->HasBallContactOccurred())
  {
    this->gameState->touchBallKickoff = this->gameState->GetLastBallContact();
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
}

// std::vector<math::Pose3<double> > initPoses;
// for (size_t i = 0; i < this->gameState->teams.size(); ++i)
// {
//   GameState::Team *currTeam = this->gameState->teams.at(i);
//   // Left team
//   if (currTeam->side == GameState::Team::Side::LEFT)
//   {
//     initPoses = SoccerField::leftKickOffPose;
//   }
//   // Right team
//   else
//   {
//     initPoses = SoccerField::rightInitPose;
//   }

//   for (size_t j = 0; j < currTeam->members.size(); ++j)
//   {
//     GameState::Agent& agent = currTeam->members.at(j);
//     this->gameState->MoveAgent(agent, initPoses.at(j).Pos(), agent.rot =
//     initPoses.at(j).Rot());
//   }
// }
