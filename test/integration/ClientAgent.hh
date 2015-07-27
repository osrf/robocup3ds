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

#ifndef _GAZEBO_INTEGRATION_TESTAGENT_HH_
#define _GAZEBO_INTEGRATION_TESTAGENT_HH_

#include <netdb.h>
#include <string>
#include <thread>
#include <ignition/math.hh>

/// \brief Class for a simple client agent to connect to the robocupplugin
/// server
class ClientAgent
{
  /// \brief Constructor for ClientAgent
  /// \param[in] _serverAddr Address of server
  /// \param[in] _port Port of that server is running on
  /// \param[in] _monitorPort Port that server is listening for monitors
  public: ClientAgent(const std::string &_serverAddr,
    const int _port, const int _monitorPort);

  /// \brief ClientAgent destructor
  public: ~ClientAgent();

  /// \brief Start the client
  public: void Start();

  /// \brief Main update loop
  public: void Update();

  /// \brief Connects agent to server
  public: bool Connect();

  /// \brief Disconnects the agent from server
  public: void Disconnect();

  /// \brief Simulates the agent moving from one location to another
  /// \param[in] _start Starting position
  /// \param[in] _end Ending position
  /// \param[in] _nSteps Number of time steps to walk
  public: void Walk(const ignition::math::Vector3<double> &_start,
    const ignition::math::Vector3<double> &_end, const int _nSteps);

  /// \brief Address of server
  private: std::string serverAddr;

  /// \brief Port of server that the client connects to
  private: const int port;

  /// \brief Port of server that the monitor connects to
  private: const int monitorPort;

  /// \brief Whether agent is running separate thread
  private: bool running;

  /// \brief Whether agent is connect to server
  private: bool connected;

  /// \brief thread in which agent is running
  private: std::thread thread;

  /// \brief Number of times left to try reconnecting
  private: int reConnects;

  /// \brief Time in milliseconds to sleep between reconnection
  private: static const int kThreadSleepTime;
};

#endif
