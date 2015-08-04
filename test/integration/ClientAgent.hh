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

#include <atomic>
#include <ignition/math.hh>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>


/// \brief Class for storing agent actions and responses from server
class ActionResponse
{
  /// \brief Status of current action
  public: enum class Status
  {
    /// \brief Not yet processed
    NOTSTARTED,
    /// \brief Currently being processed
    CURRENT,
    /// \brief Finished processing
    FINISHED
  };

  /// \brief Constructor
  /// \param[in] _actionName Name of action
  public: ActionResponse(const std::string &_actionName):
    status(Status::NOTSTARTED),
    actionName(_actionName) {}

  /// \brief Vector of messages to send to server
  public: std::vector<std::string> msgToSend;

  /// \brief Vector of monitor messages to send to server
  public: std::vector<std::string> monitorMsgToSend;

  /// \brief Vector of messages received
  public: std::vector<std::string> msgReceived;

  /// \brief Status of current message
  Status status;

  /// \brief Name of action
  std::string actionName;
};

/// \brief Class for a simple client agent to connect to the robocupplugin
/// server
class ClientAgent
{
  /// \brief Constructor for ClientAgent
  /// \param[in] _serverAddr Address of server
  /// \param[in] _port Port of that server is running on
  /// \param[in] _monitorPort Port that server is listening for monitors
  public: ClientAgent(const std::string &_serverAddr,
    const int _port, const int _monitorPort,
    const int _uNum, const std::string &_teamName,
    const std::string _side);

  /// \brief ClientAgent destructor
  public: ~ClientAgent();

  /// \brief Start the client
  public: void Start();

  /// \brief Main update loop
  public: void Update();

  /// \brief Connects agent to server
  private: bool Connect(const int &_port, int &_socketID);

  /// \brief Disconnects the agent from server
  public: void Disconnect(const int &_port, int &_socketID);

  /// \brief Adds a init and beam message to actionResponses
  /// \param[in] _x X position in meters
  /// \param[in] _y Y position in meters
  /// \param[in] _yaw Yaw in degrees
  /// \return True if action is successful
  public: void InitAndBeam(const double _x,
    const double _y, const double _yaw);

  /// \brief Simulates the agent moving from one location to another
  /// \param[in] _start Starting position
  /// \param[in] _end Ending position
  /// \param[in] _nSteps Number of time steps to walk
  public: void Walk(const ignition::math::Vector3<double> &_start,
    const ignition::math::Vector3<double> &_end, const int _nSteps);

  /// \brief Sends a change playmode monitor message
  /// \param[in] _playMode New playmode to change to
  public: void ChangePlayMode(const std::string & _playMode);

  /// \brief Sends a change move ball monitor message
  /// \param[in] _pos New position to move ball to
  public: void MoveBall(const ignition::math::Vector3<double> &_pos);

  /// \brief Sends a change move agent monitor message
  /// \param[in] _pos New position to move agent to
  public: void MoveAgent(const ignition::math::Vector3<double> &_pos);

  /// \brief Sends a monitor message to remove agent
  public: void RemoveAgent();

  /// \brief Writes to a client message to socket
  /// \param[in] _msg Message to write
  /// \return True if message is successfully written
  private: bool PutMessage(const std::string &_msg);

  /// \brief Writes a monitor message to socket
  /// \param[in] _msg Message to write
  /// \return True if message is successfully written
  private: bool PutMonMessage(const std::string &_msg);

  /// \brief Gets the message from the socket
  /// \param[out] _msg String to write message to
  private: bool GetMessage(std::string &_msg);

  /// \brief Tells us if socket is ready for reading
  /// \return True if socket is ready
  private: bool SelectInput();

  /// \brief Sleep for a certain amount of time, by default kThreadSleepTime;
  /// \param[in] Time to sleep in microseconds
  private: void Wait(const int _msec = kThreadSleepTime);

  /// \brief Whether client is running separate thread
  public: std::atomic<bool> running;

  /// \brief Whether client is connect to server
  public: std::atomic<bool> connected;

  /// Number of cycles in update()
  public: std::atomic<int> cycleCounter;

  /// \brief Unum of agent
  public: const int uNum;

  /// \brief Team name of agent
  public: const std::string teamName;

  /// \brief Side of agent
  public: const std::string side;

  /// \brief Vector of actions by client and responses by server
  public: std::vector<ActionResponse> actionResponses;

  /// \brief Vector of all the messages received by client from server
  public: std::vector<std::string> allMsgs;

  /// \brief Mutex for locking messages
  public: mutable std::mutex mutex;

  /// \brief Address of server
  private: std::string serverAddr;

  /// \brief Port of server that the client connects to
  private: const int port;

  /// \brief Port of server that the monitor connects to
  private: const int monitorPort;

  /// \brief Socket id of client
  private: int socketID;

  /// \brief Socket id of monitor
  private: int monitorSocketID;

  /// \brief thread in which agent is running
  private: std::thread thread;

  /// \brief Number of times left to try reconnecting
  private: int reConnects;

  /// \brief Time in microseconds to sleep between reconnection
  private: static const int kThreadSleepTime;

  /// \brief Specify whether to print out extra messages
  private: const int verbose;
};

#endif
