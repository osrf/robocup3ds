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
#include <vector>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/State.hh"

using namespace ignition;

/////////////////////////////////////////////////
State::State(const std::string &_name,
             GameState *_gameState)
  : name(_name), gameState(_gameState) {
  this->hasInitialized = false;
  this->isActive = false;
  this->initTime = -1;
  this->ballContactHistorySize = -1;
  this->prevState = NULL;
  this->initBallPos.Set(-999, -999, -999);
}

/////////////////////////////////////////////////
bool State::hasBallContactOccurred() {
  return this->gameState->ballContactHistory.size() > 0
         && static_cast<int>(this->gameState->ballContactHistory.size())
         > this->ballContactHistorySize;
}

/////////////////////////////////////////////////
void State::Initialize() {
  this->ballContactHistorySize =
    static_cast<int>(this->gameState->ballContactHistory.size());
  this->gameState->setBallVel(math::Vector3<double>(0, 0, 0));
  this->gameState->setBallAngVel(math::Vector3<double>(0, 0, 0));
  this->hasInitialized = true;
}

/////////////////////////////////////////////////
void State::preInitialize() {
  this->initBallPos = this->gameState->GetBall();
  this->initTime = this->gameState->getGameTime();
  this->isActive = true;
}

/////////////////////////////////////////////////
void State::unInitialize() {
  this->ballContactHistorySize = -1;
  this->initTime = -1;
  this->hasInitialized = false;
  this->isActive = false;
  this->initBallPos.Set(-999, -999, -999);
}

/////////////////////////////////////////////////
void State::Update() {
  this->gameState->CheckTiming();  // highest priority
  this->gameState->CheckDoubleTouch();  // lowest priority

  this->gameState->CheckCanScore();
  this->gameState->CheckIllegalDefense();
  this->gameState->CheckCrowding();
  this->gameState->CheckImmobility();
}

/////////////////////////////////////////////////
double State::getElapsedTime() {
  return this->gameState->getGameTime() - initTime;
}

/////////////////////////////////////////////////
std::string State::GetName() {
  return this->name;
}
