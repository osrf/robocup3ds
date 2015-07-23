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

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/KickOffState.hh"
#include "robocup3ds/states/PlayOnState.hh"

using namespace states;

/////////////////////////////////////////////////
KickOffState::KickOffState(const std::string &_name,
                           GameState *const _gameState,
                           const Team::Side _side):
  State(_name, _gameState, _side)
{
}

/////////////////////////////////////////////////
void KickOffState::Initialize()
{
  this->gameState->touchBallKickoff = NULL;
  this->gameState->ballContactHistory.clear();
  for (auto &team : this->gameState->teams)
  {
    team->canScore = false;
  }
  this->gameState->MoveBallToCenter();
  this->gameState->ReleasePlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void KickOffState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }

  // check for agents that violate sides
  gameState->CheckOffSidesOnKickOff(this->side);
  State::Update();

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsKickOff)
  {
    this->gameState->DropBallImpl(Team::Side::NEITHER);
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
  else if (this->HasBallContactOccurred())
  {
    this->gameState->touchBallKickoff = this->gameState->GetLastBallContact();
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
}
