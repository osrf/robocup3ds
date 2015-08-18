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
#include <vector>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/State.hh"

using namespace ignition;
using namespace states;

/////////////////////////////////////////////////
State::State(const std::string &_name,
             GameState *const _gameState, const Team::Side _side):
  name(_name),
  side(_side),
  gameState(_gameState),
  prevState(nullptr),
  initBallPos(math::Vector3<double>(-999, -999, -999))
{
  this->hasInitialized = false;
  this->isActive = false;
  this->initTime = -1;
  this->ballContactHistorySize = -1;
}

/////////////////////////////////////////////////
bool State::HasBallContactOccurred() const
{
  return this->gameState->ballContactHistory.size() > 0
         && static_cast<int>(this->gameState->ballContactHistory.size())
         > this->ballContactHistorySize;
}

/////////////////////////////////////////////////
void State::Initialize()
{
  this->ballContactHistorySize =
    static_cast<int>(this->gameState->ballContactHistory.size());
  this->gameState->SetBallVel(math::Vector3<double>(0, 0, 0));
  this->gameState->SetBallAngVel(math::Vector3<double>(0, 0, 0));
  this->hasInitialized = true;
}

/////////////////////////////////////////////////
void State::Preinitialize()
{
  this->initBallPos = this->gameState->GetBall();
  this->initTime = this->gameState->GetGameTime();
  this->isActive = true;
}

/////////////////////////////////////////////////
void State::Uninitialize()
{
  this->ballContactHistorySize = -1;
  this->initTime = -1;
  this->hasInitialized = false;
  this->isActive = false;
  this->initBallPos.Set(-999, -999, -999);
}

/////////////////////////////////////////////////
void State::Update()
{
  // highest priority
  this->gameState->CheckTiming();
  // lowest priority
  this->gameState->CheckDoubleTouch();

  this->gameState->CheckCanScore();
  this->gameState->CheckIllegalDefense();
  this->gameState->CheckCrowding();
  this->gameState->CheckImmobility();
}

/////////////////////////////////////////////////
double State::GetElapsedTime() const
{
  return this->gameState->GetGameTime() - this->initTime;
}

/////////////////////////////////////////////////
std::string State::GetName() const
{
  return this->name;
}
