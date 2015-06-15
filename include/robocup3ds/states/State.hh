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

#ifndef _GAZEBO_STATE_PLUGIN_HH_
#define _GAZEBO_STATE_PLUGIN_HH_

#include <string>
#include <ignition/math.hh>
#include "robocup3ds/GameState.hh"

class GameState;

// \brief State pattern used for the game mode.
class State
{
  /// \brief Class constructor.
  /// \param[in] _name Name of the state.
  /// \param[out] _gameState Reference to the GameState.
  public: State(const std::string &_name, GameState *_gameState);

  /// \brief Initialize the state. Called once when the state is entered.
  public: virtual void Initialize();

  /// \brief pre-Initialize the state. Called once when the state is entered.
  public: virtual void preInitialize();

  /// \brief Update the state.
  public: virtual void Update();

  /// \brief Get the name of the state.
  /// \brief Returns the name of the state.
  public: std::string GetName();

  /// \brief Name of the state.
  public: std::string name;
  
  /// \brief Pointer to access full game information.
  protected: GameState *gameState;
  
  /// \brief Time when we entered this game mode.
  protected: double initTime;

  /// \brief Has initialized
  protected: bool hasInitialized;

  // /// \brief Pause before calling initialized
  // protected: double initPauseTime;
  
  /// \brief Time elapsed since we entered this game mode.
  protected: double getElapsedTime();
};

#endif
