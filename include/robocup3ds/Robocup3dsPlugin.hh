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

#ifndef _GAZEBO_ROBOCUP3DS_PLUGIN_HH_
#define _GAZEBO_ROBOCUP3DS_PLUGIN_HH_

#include <map>
#include <memory>
#include <string>
#include <gazebo/gazebo.hh>

class RCPServer;
class GameState;
class Effector;
class MonitorEffector;
class Perceptor;

class Robocup3dsPlugin : public gazebo::WorldPlugin
{
  /// \brief Constructor.
  public: Robocup3dsPlugin();

  /// \brief Destructor.
  public: virtual ~Robocup3dsPlugin();

  // Documentation inherited.
  public: virtual void Load(gazebo::physics::WorldPtr _world,
      sdf::ElementPtr _sdf);

  // Documentation inherited.
  public: virtual void Init();

  /// \brief Update the robocup simulation state.
  /// \param[in] _info Information used in the update event.
  public: void Update(const gazebo::common::UpdateInfo &_info);

  /// \brief Update the robocup simulation state in sync mode
  /// \param[in] _info Information used in the update event.
  public: void UpdateSync(const gazebo::common::UpdateInfo &_info);

  /// \brief Function for loading robocupPlugin configuration variables
  /// \param[in] _config Map of configuration variables
  private: void LoadConfiguration(
    const std::map<std::string, std::string> &_config) const;

  /// \brief Update the effector, use collected joint information to update
  /// gazebo world
  private: void UpdateEffector();

  /// \brief Update the monitor effector
  private: void UpdateMonitorEffector();

  /// \brief Check if ball is colliding with any player model
  private: void UpdateBallContactHistory();

  /// \brief Sync the gameState with the gazebo world
  private: void UpdateGameState();

  /// \brief Update the effector, use gazebo world joint information to update
  /// information sent to agents
  private: void UpdatePerceptor();

  /// \brief Port used for connecting client agents
  public: static int clientPort;

  /// \brief Port used for connecting monitors
  public: static int monitorPort;

  /// \brief Flag whether to enable sync mode, with this enable, the plugin
  /// trys to update as fast as possible
  public: static bool syncMode;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to world
  private: gazebo::physics::WorldPtr world;

  /// \brief Pointer to sdf
  private: sdf::ElementPtr sdf;

  /// \brief Pointer to GameState object
  private: std::shared_ptr<GameState> gameState;

  /// \brief Pointer to Effector object;
  private: std::shared_ptr<Effector> effector;

  /// \brief Pointer to Monitor Effector object;
  private: std::shared_ptr<MonitorEffector> monitorEffector;

  /// \brief Pointer to Perceptor object
  private: std::shared_ptr<Perceptor> perceptor;

  /// \brief Pointer to server for clients
  private: std::shared_ptr<RCPServer> clientServer;

  /// \brief Pointer to server for monitors
  private: std::shared_ptr<RCPServer> monitorServer;

  /// \brief Pointer to buffer for sending messages to server;
  private: char* buffer;

  /// \brief Gazebo simulation time when last update occurred
  private: double lastUpdateTime;

  /// \brief Size of buffer in bytes
  private: static const int kBufferSize;
};

#endif
