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

#ifndef _GAZEBO_ROBOCUP3DS_PLAY_STATE_HH_
#define _GAZEBO_ROBOCUP3DS_PLAY_STATE_HH_

#include <string>

#include "robocup3ds/states/State.hh"

class GameState;

namespace states
{
  /// \brief State that handles regular gameplay.
  class PlayOnState : public State
  {
    /// \brief Class constructor.
    /// \param[in] _name Name of the state.
    /// \param[in] _gameState Reference to the GameState
    public: PlayOnState(const std::string &_name,
                        GameState *const _gameState);

    // Documentation inherited
    public: virtual void Initialize();

    // Documentation inherited
    public: virtual void Update();
  };
}

#endif
