/*
 * Copyright (C) 2014 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may !use this file except in compliance with the License.
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

using namespace ignition;

/////////////////////////////////////////////////
BeforeKickOffState::BeforeKickOffState(const std::string &_name,
                                       GameState *_gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void BeforeKickOffState::Initialize()
{
  this->gameState->StopPlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void BeforeKickOffState::Update()
{
  if (!hasInitialized) {
     this->Initialize();
  }
  if (this->gameState->GetBall() != SoccerField::BallCenterPosition) {
    this->gameState->MoveBallToCenter();
  }

  // resets getElapsedGameTime() back to zero
  this->gameState->setStartGameTime(this->gameState->getGameTime());

  // After some time, go to play mode.
  if (this->getElapsedTime() >= GameState::SecondsBeforeKickOff) {
    if (this->gameState->GetHalf() == GameState::FIRST_HALF) {
      this->gameState->SetCurrent(this->gameState->kickOffLeftState.get());
    }
    else
    {
      this->gameState->SetCurrent(this->gameState->kickOffRightState.get());
    }
  }
}
