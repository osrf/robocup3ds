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

#ifndef _GAZEBO_ROBOCUP3DS_STATE_PLUGIN_HH_
#define _GAZEBO_ROBOCUP3DS_STATE_PLUGIN_HH_

#include <memory>
#include <string>
#include <ignition/math.hh>

#include "robocup3ds/Agent.hh"

class GameState;

namespace states
{
  /// \brief State pattern used for the game modes.
  class State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[in] _gameState Reference to the GameState
    /// \param[in] _side Side of state
    public: State(const std::string &_name,
                  GameState *const _gameState,
                  const Team::Side _side = Team::Side::NEITHER);

    /// \brief Initialize the state. Called once after a pause duration after
    /// entering state.
    public: virtual void Initialize();

    /// \brief Preinitialize the state. Called once when the state is entered.
    public: virtual void Preinitialize();

    /// \brief Uninitialize the state. Called once when leaving current state
    /// and switching to another.
    public: virtual void Uninitialize();

    /// \brief Update the state.
    public: virtual void Update();

    /// \brief Returns the name of the state
    /// \return Name of state
    public: std::string GetName() const;

    /// \brief Checks whether agent contacts ball since Initialize() is called
    /// \return True if agent has contacted ball
    public: bool HasBallContactOccurred() const;

    /// \brief Time elapsed since we entered this game mode.
    /// \return Seconds since current playmode begin
    public: double GetElapsedTime() const;

    /// \brief Used to determine if ball contact has occurred since
    /// Initialize() is called
    public: int ballContactHistorySize;

    /// \brief Name of the state.
    public: const std::string name;

    /// \brief Side that this state belongs to
    public: const Team::Side side;

    /// \brief Pointer to access full game information.
    public: GameState *const gameState;

    /// \brief Time when we entered this game mode.
    public: double initTime;

    /// \brief Has initialized
    public: bool hasInitialized;

    /// \brief Whether this is the state that gameState->currentState is
    /// set to or not
    public: bool isActive;

    /// \brief Pointer to previous state
    public: std::shared_ptr<State> prevState;

    /// \brief Position of ball when we enter this state
    public: ignition::math::Vector3<double> initBallPos;
  };
}

#endif
