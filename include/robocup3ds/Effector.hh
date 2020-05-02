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
#include <mutex>
#include <sstream>
#include <string>
#include <vector>
#include "robocup3ds/SocketParser.hh"
#include "sexpLibrary/sexp.h"
#include "sexpLibrary/sexp_ops.h"

#include "robocup3ds/Agent.hh"

class GameState;
class NaoBT;

/// \brief This is a Effector class. It implemented Parse() method
/// inherited from SocketParser class. Parse() method has been used
/// to receive messages from the socket.Retrieving effectors values
/// from received messages have been implemented using s-expression
/// library.
class Effector: public SocketParser
{
  /// \brief Class constructor.
  /// \param[in] _gameState Pointer to GameState object
  public: Effector(GameState *const _gamestate);

  /// \brief Class destructor.
  public: ~Effector();

  /// \brief Used to read incoming message received by socket.
  /// \param[in] _socket, Socket of the client should be assigned by server.
  /// \return True when success or false otherwise.
  public: bool Parse(const int _socket);

  /// \brief Update obtained effectors values using s-expression library and
  /// set them in agent.action
  public: virtual void Update();

  /// \brief Used in server class constructor as a callback function.
  /// \param[in] _socket the socket used for the server client communication.
  public: void OnConnection(const int _socket);

  /// \brief Used in server class constructor.
  /// \param[in] the socket used for for server client communication.
  public: void OnDisconnection(const int _socket);

  /// \brief Main procedure of extracting information
  /// in pile of S-expressions using the expression library.
  /// \param[in] _msg S-expression messages.
  protected: void ParseMessage(const std::string &_msg);

  /// \brief Used to parse each S-expression message.
  /// \param[in] _exp pointer to a S-expression.
  protected: virtual void ParseSexp(sexp_t *_exp);

  /// \brief Used to retrieve information from Scene effector message.
  /// \param[in] _exp Pointer to a S-expression message.
  protected: void ParseScene(sexp_t *_exp);

  /// \brief Used to retrieve information from Beam effector message.
  /// \param[in] _exp Pointer to a S-expression message.
  protected: void ParseBeam(sexp_t *_exp);

  /// \brief Used to retrieve information from Init message.
  /// \param[in] _exp Pointer to a S-expression.
  protected: void ParseInit(sexp_t *_exp);

  /// \brief Used to retrieve information from Say message. Accept the
  /// say Message which consist of at most 20 ASCII printing characters
  /// \param[in] _exp Pointer to a S-expression.
  protected: void ParseSay(sexp_t *_exp);

  /// \brief Used to parse the joints effector value in S-expression messages.
  /// \param[in] _exp Pointer to a S-expression.
  protected: void ParseHingeJoint(sexp_t *_exp);

  /// \brief List of agents to add to gazebo world this update cycle.
  public: std::vector<Agent*> agentsToAdd;

  /// \brief List of agent names to remove from gazebo world this update cycle.
  public: std::vector<std::string> agentsToRemove;

  /// \brief List of sockets to disconnect this update cycle
  public: std::vector<int> socketsToDisconnect;

  /// \brief Map of socket ids and agent body types strings
  public: std::map<int, std::shared_ptr<NaoBT>> socketIDbodyTypeMap;

  /// \brief Maximum size of each message received.
  protected: static const int kBufferSize = 16384;

  /// \brief Pointer to gameState object
  protected: GameState *const gameState;

  /// \brief Protect concurrent access.
  protected: mutable std::mutex mutex;

  /// \brief Data Structure used to store Message received by sockets.
  /// Here, the key of the map is the Socket IDs which is assigned in
  /// OnConnection()
  protected: std::map<int, std::string> socketIDMessageMap;

  /// \brief Buffer for reading from socket
  protected: char buffer[Effector::kBufferSize] = {0};

  /// \brief Buffer for reading from socket
  protected: char sexpBuffer[Effector::kBufferSize]= {0};

  /// \brief Pointer to current agent whose message is being parsed
  protected: Agent* currAgent;

  /// \brief The socketID of currAgent
  protected: int currSocketId;
};

class MonitorEffector : public Effector
{
  /// \brief Class constructor.
  /// \param[in] _gameState Pointer to GameState object
  public: MonitorEffector(GameState *const _gamestate);

  /// \brief Iterate through all monitor messages and parse them
  public: void Update();

  /// \brief Used to parse each S-expression message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseSexp(sexp_t *_exp);

  /// \brief Used to parse each move agent S-expression
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseMoveAgent(sexp_t *_exp);

  /// \brief Used to parse each move ball S-expression
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseMoveBall(sexp_t *_exp);

  /// \brief Used to parse each change playMode S-expression.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParsePlayMode(sexp_t *_exp);

  /// \brief Used to parse each remove agent S-expression.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseRemoveAgent(sexp_t *_exp);
};

#endif /* _GAZEBO_ROBOCUP3DS_EFFECTOR_HH_ */
