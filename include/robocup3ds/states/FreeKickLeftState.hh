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

#ifndef _GAZEBO_FREE_KICK_LEFT_STATE_HH_
#define _GAZEBO_FREE_KICK_LEFT_STATE_HH_

#include <string>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/State.hh"

/// \class FreeKickLeftState FreeKickLeftState.hh
/// robocup3ds/states/FreeKickLeftState.hh
/// \brief State that handles the free kick left state.
class FreeKickLeftState : public State
{
  // Documentation inherited
  public: FreeKickLeftState(const std::string &_name,
                            GameState *_gameState);

  // Documentation inherited
  public: virtual void Initialize();

  // Documentation inherited
  public: virtual void Update();

  /// \brief Set the position where the ball will be located.
  /// \param[in] _ballPos Desired ball position.
  public: void SetPos(const ignition::math::Vector3<double> &_pos);

  /// \brief Used to select the position where the ball will be moved.
  private: ignition::math::Vector3<double> pos;
};

#endif
