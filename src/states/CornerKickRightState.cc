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
#include "robocup3ds/states/CornerKickRightState.hh"
#include "robocup3ds/states/PlayOnState.hh"

using namespace states;

/////////////////////////////////////////////////
CornerKickRightState::CornerKickRightState(const std::string &_name,
    GameState *const _gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void CornerKickRightState::Initialize()
{
  this->gameState->MoveBall(initBallPos);
  this->gameState->MoveBallToCorner();
  State::Initialize();
}

/////////////////////////////////////////////////
void CornerKickRightState::Update()
{
  if (this->GetElapsedTime() < GameState::SecondsKickInPause)
  {
    return;
  }
  else if (!this->hasInitialized)
  {
    this->Initialize();
  }

  // The left team is not allowed to be close to the ball.
  this->gameState->DropBallImpl(GameState::Team::Side::RIGHT);
  State::Update();

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsKickIn)
  {
    this->gameState->DropBallImpl(GameState::Team::Side::NEITHER);
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
  else if (this->HasBallContactOccurred())
  {
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
}
