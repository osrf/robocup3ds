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
#include <vector>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/State.hh"

using namespace ignition;

/////////////////////////////////////////////////
State::State(const std::string &_name,
             GameState *_gameState)
  : name(_name), gameState(_gameState)
{
  hasInitialized = false;
  isActive = false;
  initTime = -1;
  ballContactHistorySize = -1;
  prevState = NULL;
  initBallPos.Set(-999,-999,-999);
}

/////////////////////////////////////////////////
bool State::hasBallContactOccurred()
{
  return gameState->ballContactHistory.size() > 0 and static_cast<int>(gameState->ballContactHistory.size()) > ballContactHistorySize;
}

/////////////////////////////////////////////////
void State::Initialize()
{
  ballContactHistorySize = static_cast<int>(gameState->ballContactHistory.size());
  gameState->setBallVel(math::Vector3<double>(0, 0, 0));
  gameState->setBallAngVel(math::Vector3<double>(0, 0, 0));
  hasInitialized = true;
}

/////////////////////////////////////////////////
void State::preInitialize()
{
  initBallPos = gameState->GetBall();
  initTime = gameState->getGameTime();
  isActive = true;
}

/////////////////////////////////////////////////
void State::unInitialize()
{
  ballContactHistorySize = -1;
  initTime = -1;
  hasInitialized = false;
  isActive = false;
  initBallPos.Set(-999,-999,-999);
}

/////////////////////////////////////////////////
void State::Update()
{
  gameState->CheckTiming(); //highest priority
  gameState->CheckDoubleTouch(); //lowest priority

  gameState->CheckCanScore();
  gameState->CheckIllegalDefense();
  gameState->CheckCrowding();
  gameState->CheckImmobility();
}

/////////////////////////////////////////////////
double State::getElapsedTime()
{
  return gameState->getGameTime() - initTime;
}

/////////////////////////////////////////////////
std::string State::GetName()
{
  return name;
}
