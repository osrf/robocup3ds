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
#include <ignition/math.hh>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/GoalKickState.hh"
#include "robocup3ds/states/PlayOnState.hh"

using namespace states;
using namespace ignition;

/////////////////////////////////////////////////
GoalKickState::GoalKickState(const std::string &_name,
                             GameState *const _gameState,
                             const Team::Side _side):
  State(_name, _gameState, _side)
{
}

/////////////////////////////////////////////////
void GoalKickState::Initialize()
{
  // Move the ball.
  this->gameState->MoveBall(initBallPos);
  this->gameState->MoveBallForGoalKick();
  State::Initialize();
}

/////////////////////////////////////////////////
void GoalKickState::Update()
{
  if (this->GetElapsedTime() < GameState::SecondsKickInPause)
  {
    return;
  }
  else if (!this->hasInitialized)
  {
    this->Initialize();
  }

  this->gameState->DropBallImpl(this->side);
  this->gameState->CheckGoalKickIllegalDefense(this->side);
  math::Box penaltyBox;
  if (this->side == Team::Side::LEFT)
  {
    penaltyBox = SoccerField::PenaltyBoxLeft;
  }
  else
  {
    penaltyBox = SoccerField::PenaltyBoxRight;
  }
  State::Update();

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsKickIn)
  {
    this->gameState->DropBallImpl(Team::Side::NEITHER);
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
  else if (!penaltyBox.Contains(this->gameState->GetBall()))
  {
    this->gameState->SetCurrent(this->gameState->playOnState);
  }
}
