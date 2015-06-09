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

#ifndef _GAZEBO_ROBOCUP_3DS_PLUGIN_HH_
#define _GAZEBO_ROBOCUP_3DS_PLUGIN_HH_

#include <gazebo/gazebo.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

#include "robocup3ds/states/BeforeKickOffState.hh"
#include "robocup3ds/states/CornerKickLeftState.hh"
#include "robocup3ds/states/CornerKickRightState.hh"
#include "robocup3ds/states/FreeKickLeftState.hh"
#include "robocup3ds/states/FreeKickRightState.hh"
#include "robocup3ds/states/GameOverState.hh"
#include "robocup3ds/states/GoalKickLeftState.hh"
#include "robocup3ds/states/GoalKickRightState.hh"
#include "robocup3ds/states/GoalLeftState.hh"
#include "robocup3ds/states/GoalRightState.hh"
#include "robocup3ds/states/KickInLeftState.hh"
#include "robocup3ds/states/KickInRightState.hh"
#include "robocup3ds/states/KickOffLeftState.hh"
#include "robocup3ds/states/KickOffRightState.hh"
#include "robocup3ds/states/PlayState.hh"

namespace gazebo
{
class Robocup3dsPlugin : public WorldPlugin
{
public:
  static const std::string BeforeKickOff;
  static const std::string KickOffLeft;
  static const std::string KickOffRight;
  static const std::string Play;
  static const std::string KickInLeft;
  static const std::string KickInRight;
  static const std::string CornerKickLeft;
  static const std::string CornerKickRight;
  static const std::string GoalKickLeft;
  static const std::string GoalKickRight;
  static const std::string GameOver;
  static const std::string GoalLeft;
  static const std::string GoalRight;
  static const std::string FreeKickLeft;
  static const std::string FreeKickRight;
  boost::shared_ptr<BeforeKickOffState> beforeKickOffState;
  boost::shared_ptr<KickOffLeftState> kickOffLeftState;
  boost::shared_ptr<KickOffRightState> kickOffRightState;
  boost::shared_ptr<PlayState> playState;
  boost::shared_ptr<KickInLeftState> kickInLeftState;
  boost::shared_ptr<KickInRightState> kickInRightState;
  boost::shared_ptr<CornerKickLeftState> cornerKickLeftState;
  boost::shared_ptr<CornerKickRightState> cornerKickRightState;
  boost::shared_ptr<GoalKickLeftState> goalKickLeftState;
  boost::shared_ptr<GoalKickRightState> goalKickRightState;
  boost::shared_ptr<GameOverState> gameOverState;
  boost::shared_ptr<GoalLeftState> goalLeftState;
  boost::shared_ptr<GoalRightState> goalRightState;
  boost::shared_ptr<FreeKickLeftState> freeKickLeftState;
  boost::shared_ptr<FreeKickRightState> freeKickRightState;

  /// \brief (left or right, player_name).
  std::pair<int, std::string> lastPlayerTouchedBall;
  /// \brief Pointer to the world.
  physics::WorldPtr world;
  /// \brief Pointer to the soccer ball.
  physics::ModelPtr ball;

  /// \brief Constructor.
public: Robocup3dsPlugin();

  /// \brief Destructor.
public: virtual ~Robocup3dsPlugin();

  // Documentation inherited.
public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

  // Documentation inherited.
public: virtual void Init();

  /// \brief Update the robocup simulation state.
  /// \param[in] _info Information used in the update event.
public: void Update(const common::UpdateInfo &_info);

  /// \brief Pointer to the update event connection.
private: event::ConnectionPtr updateConnection;
};
}
#endif
