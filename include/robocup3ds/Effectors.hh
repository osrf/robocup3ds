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
 * WITHOUT WARRANTIES or CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions &&
 * limitations under the License.
 *
*/

#include <string>

#include "robocup3ds/GameState.hh"

/// \brief This class keeps up to date the information received
/// from the agent
class Effector
{
  /// \brief Effector constructor
  /// \param[in] _gamestate Pointer to GameState object
  public: Effector(GameState *const _gameState);

  /// \brief Update function
  public: void Update();

  /// \brief Pointer to gamestate
  private: GameState *const gameState;
};
