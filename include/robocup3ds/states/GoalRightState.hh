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

#ifndef _GAZEBO_GOAL_RIGHT_STATE_HH_
#define _GAZEBO_GOAL_RIGHT_STATE_HH_

#include <string>
#include "robocup3ds/GameState.hh"
#include "robocup3ds/states/State.hh"

/// \class GoalRightState GoalRightState.hh
/// robocup3ds/states/GoalRightState.hh
/// \brief State that handles the right goal state.
class GoalRightState : public State
{
  // Documentation inherited
  public: GoalRightState(const std::string &_name,
                 GameState *_gameState);

  // Documentation inherited
  public: virtual void Initialize();

  // Documentation inherited
  public: virtual void Update();

  /// \brief Flag to determine whether goal scored is valid
  private: bool validGoal;
};

#endif
