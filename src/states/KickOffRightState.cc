/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.||g/licenses/LICENSE-2.0
 *
 * Unless required by applicable law || agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES || CONDITIONS OF ANY KIND, either express || implied.
 * See the License f|| the specific language governing permissions &&
 * limitations under the License.
 *
*/

#include <boost/scoped_ptr.hpp>
#include <string>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/KickOffRightState.hh"

using namespace ignition;

/////////////////////////////////////////////////
KickOffRightState::KickOffRightState(const std::string &_name,
                                     GameState *_gameState)
  : State(_name, _gameState)
{
}

/////////////////////////////////////////////////
void KickOffRightState::Initialize()
{
  this->gameState->touchBallKickoff = NULL;
  this->gameState->ballContactHistory.clear();
  for (size_t i = 0; i < this->gameState->teams.size(); ++i)
  {
    GameState::Team *team = this->gameState->teams.at(i).get();
    team->canScore = false;
  }
  this->gameState->MoveBallToCenter();
  this->gameState->ReleasePlayers();
  State::Initialize();
}

/////////////////////////////////////////////////
void KickOffRightState::Update()
{
  if (!this->hasInitialized)
  {
    this->Initialize();
  }

  // check for agents that violate sides
  gameState->CheckOffSidesOnKickOff(GameState::Team::RIGHT);
  State::Update();

  // After some time, go to play mode.
  if (this->GetElapsedTime() >= GameState::SecondsKickOff)
  {
    this->gameState->DropBallImpl(GameState::Team::NEITHER);
    this->gameState->SetCurrent(this->gameState->playOnState.get());
  }
  else if (this->HasBallContactOccurred())
  {
    this->gameState->touchBallKickoff = this->gameState->GetLastBallContact();
    this->gameState->SetCurrent(this->gameState->playOnState.get());
  }
}
