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
#include "robocup3ds/states/GoalKickLeftState.hh"

using namespace ignition;

/////////////////////////////////////////////////
GoalKickLeftState::GoalKickLeftState(const std::string &_name,
                                     GameState *_gameState)
  : State(_name, _gameState) {
}

/////////////////////////////////////////////////
void GoalKickLeftState::Initialize() {
  // Move the ball.
  gameState->MoveBall(initBallPos);
  gameState->MoveBallForGoalKick();
  State::Initialize();
}

/////////////////////////////////////////////////
void GoalKickLeftState::Update() {
  if (getElapsedTime() < GameState::SecondsKickInPause) {
    return;
  }
  else if (!hasInitialized)
  {
    Initialize();
  }

  gameState->DropBallImpl(GameState::Team::LEFT);
  gameState->CheckGoalKickIllegalDefense(GameState::Team::LEFT);
  State::Update();

  // After some time, go to play mode.
  if (getElapsedTime() >= GameState::SecondsKickIn) {
    gameState->DropBallImpl(GameState::Team::NEITHER);
    gameState->SetCurrent(gameState->playState.get());
  }
  else if (!SoccerField::PenaltyBoxLeft.Contains(gameState->GetBall()))
  {
    gameState->SetCurrent(gameState->playState.get());
  }
}
