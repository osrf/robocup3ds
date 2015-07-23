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

#ifndef _GAZEBO_ROBOCUP3DS_EFFECTOR_HH_
#define _GAZEBO_ROBOCUP3DS_EFFECTOR_HH_

#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "robocup3ds/SocketParser.hh"
#include "../../lib/sexpLibrary/sexp.h"
#include "../../lib/sexpLibrary/sexp_ops.h"

#include "robocup3ds/Agent.hh"

class GameState;

/// \brief This is a Effector class. It implemented Parse() method
/// inherited from SocketParser class. Parse() method has been used
/// to receive messages from the socket.Retrieving effectors values
/// from received messages have been implemented using s-expression
/// library.
class Effector: public SocketParser
{
  /// \brief Class constructor.
  public: Effector(GameState *const _gamestate);

  /// \brief Class destructor.
  public: ~Effector();

  /// \brief Used to read incoming message received by socket.
  /// \param[in] _socket, Socket of the client should be assigned by server.
  /// \return True when success or false otherwise.
  public: bool Parse(const int _socket);

  /// \brief Update obtained effectors values using s-expression library and
  /// set them in agent.action
  public: void Update();

  /// \brief Used in server class constructor as a callback function.
  /// \param[in] _socket the socket used for the server client communication.
  public: void OnConnection(const int _socket);

  /// \brief Used in server class constructor.
  /// \param[in] the socket used for for server client communication.
  public: void OnDisconnection(const int _socket);

  /// \brief Main procedure of extracting information
  /// in pile of S-expressions using the expression library.
  /// \param[in] _msg S-expression messages.
  private: void ParseMessage(const std::string &_msg);

  /// \brief Used to parse each S-expression message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseSexp(sexp_t *_exp);

  /// \brief Used to retrieve information from Scene effector message.
  /// \param[in] _exp Pointer to a S-expression message.
  private: void ParseScene(sexp_t *_exp);

  /// \brief Used to retrieve information from Beam effector message.
  /// \param[in] _exp Pointer to a S-expression message.
  private: void ParseBeam(sexp_t *_exp);

  /// \brief Used to retrieve information from Init message.
  /// \param[in] _exp Pointer to a S-expression.
  private: void ParseInit(sexp_t *_exp);

  /// \brief Used to parse the joints effector value in S-expression messages.
  /// \param[in] _exp Pointer to a S-expression.
  private: void ParseHingeJoint(sexp_t *_exp);

  /// \brief data structure used for init information.
  public: std::vector<AgentId> agentsToAdd;

  /// \brief data structure used for init information.
  public: std::vector<AgentId> agentsToRemove;

  /// \brief Data Structure used to store Message received by sockets.
  /// Here, the key of the map is the Socket IDs which is assigned in
  /// OnConnection()
  private: std::map<int, std::string> socketIDMessageMap;

  /// \brief Maximum size of each message received.
  private: static const int kBufferSize;

  /// \brief Protect concurrent access.
  private: mutable std::mutex mutex;

  /// \brief Buffer for reading from socket
  private: char *buffer;

  /// \brief Buffer for reading from socket
  private: char *sexpBuffer;

  /// \brief Pointer to current agent whose message is being parsed
  private: Agent* currAgent;

  /// \brief The socketID of currAgent
  private: int currSocketId;

  /// \brief Pointer to gameState object
  private: GameState *const gameState;
};

#endif /* _GAZEBO_ROBOCUP3DS_EFFECTOR_HH_ */
