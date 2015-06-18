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
#include "robocup3ds/states/GoalLeftState.hh"

using namespace ignition;

/////////////////////////////////////////////////
GoalLeftState::GoalLeftState(const std::string &_name,
                             GameState *_gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void GoalLeftState::Initialize()
{
  validGoal = true;

  // Register the left team goal.
  for (size_t i = 0; i < gameState->teams.size(); ++i) {
    GameState::Team *team = gameState->teams.at(i);
    if (team->side == GameState::Team::LEFT) {
      if (team->canScore) {
        team->score++;
      } else {
        validGoal = false;
      }
    }
  }
  State::Initialize();
}

/////////////////////////////////////////////////
void GoalLeftState::Update()
{
  if (!hasInitialized) {
    Initialize();
  }
  // Afer some time, go to right team kick off mode.
  if (getElapsedTime() >= GameState::SecondsGoalPause or !validGoal) {
    gameState->SetCurrent(gameState->kickOffRightState.get());
  }
}
