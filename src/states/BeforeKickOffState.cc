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
#include "robocup3ds/states/BeforeKickOffState.hh"
#include "robocup3ds/states/KickOffState.hh"

using namespace states;

/////////////////////////////////////////////////
BeforeKickOffState::BeforeKickOffState(const std::string &_name,
                                       GameState *const _gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void BeforeKickOffState::Initialize()
{
  State::Initialize();
}

/////////////////////////////////////////////////
void BeforeKickOffState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }

  bool agentOnField = false;
  for (const auto &team : this->gameState->teams)
  {
    if (team->members.size() > 0u)
    { agentOnField = true; }
  }
  if (!agentOnField)
  { this->initTime = this->gameState->GetGameTime(); }

  // resets getElapsedGameTime() back to zero
  this->gameState->SetStartGameTime(this->gameState->GetGameTime());

  this->gameState->StopPlayers();
  if (this->gameState->GetBall() != SoccerField::kBallCenterPosition)
  {
    this->gameState->MoveBallToCenter();
  }

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsBeforeKickOff)
  {
    if (this->gameState->GetHalf() == GameState::Half::FIRST_HALF)
    { this->gameState->SetCurrent(this->gameState->kickOffLeftState); }
    else
    { this->gameState->SetCurrent(this->gameState->kickOffRightState); }
  }
}
