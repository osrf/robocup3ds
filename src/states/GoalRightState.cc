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
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/GoalRightState.hh"

using namespace ignition;

/////////////////////////////////////////////////
GoalRightState::GoalRightState(const std::string &_name,
                               GameState *_gameState)
	: State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void GoalRightState::Initialize()
{
	State::Initialize();
  validGoal = true;

	// Register the right team goal.
  for (size_t i = 0; i < gameState->teams.size(); ++i) {
    GameState::Team *team = gameState->teams.at(i);
    if (team->side == GameState::Team::RIGHT) {
      if (team->canScore) {
        team->score++;
      } else {
        validGoal = false;
      }
    }
  }
}

/////////////////////////////////////////////////
void GoalRightState::Update()
{
	if (not hasInitialized) {
		Initialize();
	}
	// After some time, go to left team kick off mode.
	if (getElapsedTime() > GameState::SecondsGoalPause or not validGoal) {
		gameState->SetCurrent(gameState->kickOffLeftState.get());
	}
}
