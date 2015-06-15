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
  initTime = -1;
}

/////////////////////////////////////////////////
void State::Initialize()
{
  // std::cout << "New state: " << name << std::endl;
  gameState->clearBallContactHistory();

  gameState->setBallVel(math::Vector3<double>(0, 0, 0));
  gameState->setBallAngVel(math::Vector3<double>(0, 0, 0));

  hasInitialized = true;
}

/////////////////////////////////////////////////
void State::preInitialize()
{
  initTime = gameState->getElapsedGameTime();
  hasInitialized = false;
}

/////////////////////////////////////////////////
void State::Update()
{
  gameState->CheckTiming();
  gameState->CheckIllegalDefense();
  gameState->CheckCrowding();
  gameState->CheckImmobility();
}

/////////////////////////////////////////////////
double State::getElapsedTime()
{
  return gameState->getElapsedGameTime() - initTime;
}

/////////////////////////////////////////////////
std::string State::GetName()
{
  return name;
}
