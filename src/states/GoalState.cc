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
#include <gazebo/gazebo.hh>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/GoalState.hh"
#include "robocup3ds/states/KickOffState.hh"

using namespace states;

/////////////////////////////////////////////////
GoalState::GoalState(const std::string &_name,
                     GameState *const _gameState,
                     const Team::Side _side):
  State(_name, _gameState, _side)
{
  this->validGoal = false;
}

/////////////////////////////////////////////////
void GoalState::Initialize()
{
  this->validGoal = true;

  // Register the left team goal.
  gzmsg << "ball in goal, current score:" << std::endl;
  for (auto &team : this->gameState->teams)
  {
    if (team->side == this->side)
    {
      if (team->canScore)
      {
        team->score++;
      }
      else
      {
        this->validGoal = false;
      }
      gzmsg << team->name << ": " << team->score << std::endl;
    }
  }
  State::Initialize();
}

/////////////////////////////////////////////////
void GoalState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }
  // Afer some time, go to kick off mode.
  if (this->GetElapsedTime() >= GameState::SecondsGoalPause || !this->validGoal)
  {
    if (this->side == Team::Side::LEFT)
    {
      this->gameState->SetCurrent(this->gameState->kickOffRightState);
    }
    else
    {
      this->gameState->SetCurrent(this->gameState->kickOffLeftState);
    }
  }
}
