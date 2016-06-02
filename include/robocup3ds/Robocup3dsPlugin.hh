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
#include <vector>
#include <gazebo/gazebo.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <sdf/sdf.hh>

class RCPServer;
class GameState;
class Effector;
class MonitorEffector;
class Perceptor;
class Agent;

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
  public: virtual void Update(const gazebo::common::UpdateInfo &_info);

  /// \brief Update the robocup simulation state in sync mode
  /// \param[in] _info Information used in the update event.
  public: virtual void UpdateSync(const gazebo::common::UpdateInfo &_info);

  /// \brief Publish game state information to GUI plugin
  private: void PublishGameInfo();

  /// \brief Function for loading robocupPlugin configuration variables
  /// \param[in] _config Map of configuration variables
  private: void LoadConfiguration(
    const std::map<std::string, std::string> &_config);

  /// \brief Helper function to load custom gains and limits for PID controllers
  /// of each body type and joint
  /// \param[out] _pid PIDs whose params to set
  /// \param[in] _bodyType Name of bodytype
  /// \param[in] _jointName Name of joint
  /// \param[in] _config Map of configuration variables
  private: void LoadPIDParams(gazebo::common::PID &_pid,
                              const std::string &_bodyType,
                              const std::string &_jointName,
                              const std::map<std::string,
                                             std::string> &_config) const;

  /// \brief Copies contact objects from the world contact manager to contacts
  private: void UpdateContactManager();

  /// \brief Update the effector, use collected joint information to update
  /// gazebo world
  private: void UpdateEffector();

  /// \brief Sets the pids for the joint controller of agent's model
  /// \param[in] _agent Agent object
  /// \param[in] _model Pointer to agent's model
  private: void InitJointController(const Agent &_agent,
                                    const gazebo::physics::ModelPtr &_model);

  /// \brief Update the monitor effector
  private: void UpdateMonitorEffector();

  /// \brief Check if ball is colliding with any player model
  private: void UpdateBallContactHistory();

  /// \brief Sync the gameState with the gazebo world
  private: void UpdateGameState();

  /// \brief Update the effector, use gazebo world joint information to update
  /// information sent to agents
  private: void UpdatePerceptor();

  /// \brief Fix agents in place if they are stopped
  private: void UpdateStoppedAgents();

  /// \brief Update the playmode based on msgs sent by the GUI plugin
  /// \param[in] _msg Message received from GUI
  private: void UpdateGUIPlaymode(ConstGzStringPtr &_msg);

  /// \brief Port used for connecting client agents
  public: int clientPort = 3100;

  /// \brief Port used for connecting monitors
  public: int monitorPort = 3200;

  /// \brief Flag whether to enable sync mode, with this enable, the plugin
  /// tries to update as fast as possible
  public: bool syncMode = false;

  /// \brief Pointer to world
  protected: gazebo::physics::WorldPtr world;

  /// \brief Pointer to GameState object
  protected: std::shared_ptr<GameState> gameState;

  /// \brief Size of buffer in bytes
  private: static const int kBufferSize = 16384;

  /// \brief Integral limit used for all joint controllers
  private: static const double kPID_I_LIMIT;

  /// \brief Output limit used for all joint controllers
  private: static const double kPID_CMD_LIMIT;

  /// \brief Pointer to the update event connection.
  private: gazebo::event::ConnectionPtr updateConnection;

  /// \brief Pointer to sdf
  private: sdf::ElementPtr sdf;

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
  private: char buffer[kBufferSize];

  /// \brief Gazebo simulation time when last update occurred
  private: double lastUpdateTime;

  /// \brief Vector of all contacts received from contact manager
  private: std::vector<gazebo::physics::Contact> contacts;

  /// \brief Game state publisher.
  private: gazebo::transport::PublisherPtr statePub;

  /// \brief Pointer to a node for communication.
  private: gazebo::transport::NodePtr gzNode;

  /// \brief Subscriber to playmode change messge from GUI plugin.
  private: gazebo::transport::SubscriberPtr playmodeSub;
};

#endif
