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
#include "robocup3ds/states/PlayOnState.hh"

using namespace states;

/////////////////////////////////////////////////
PlayOnState::PlayOnState(const std::string &_name,
                         GameState *const _gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void PlayOnState::Initialize()
{
  this->gameState->ReleasePlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void PlayOnState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }
  this->gameState->CheckCanScore();

  this->gameState->CheckTiming();  // highest priority
  this->gameState->CheckDoubleTouch();
  this->gameState->CheckBall();  // lowest priority

  this->gameState->CheckIllegalDefense();
  this->gameState->CheckCrowding();
  this->gameState->CheckImmobility();
}
