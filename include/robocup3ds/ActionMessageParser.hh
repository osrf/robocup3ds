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

#ifndef _GAZEBO_ROBOCUP3DS_ACTIONMESSAGEPARSER_HH_
#define _GAZEBO_ROBOCUP3DS_ACTIONMESSAGEPARSER_HH_

#include <sstream>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#include <memory>
#include "../../src/sexpLibrary/sexp.h"
#include "../../src/sexpLibrary/sexp_ops.h"
#include "robocup3ds/SocketParser.hh"

class ActionMessageParser: public gazebo::SocketParser
{

  /// \brief SceneMsg class contains information of scene message
  class SceneMsg
  {
    /// \brief Constructor.
    /// \param[in] _agentId the sockets.
    /// \param[in] _robotType robot type in Scene message.
    /// \param[in] _rsgAddress rsg file address.
    public: SceneMsg(int _agentId, int _robotType, std::string _rsgAddress)
    {
      this->id = _agentId;
      this->robotType = _robotType;
      this->rsgAddress = _rsgAddress;
    }

    public: int id;

    public: std::string rsgAddress;

    public: int robotType;
  };

  /// \brief BeamMsg class contains information of beam message
  class BeamMsg
  {
    /// \brief Constructor.
    /// \param[in] _agentId the socket ID.
    /// \param[in] _x, _y, _z Beam Position
    public: BeamMsg(int _agentId, double _x, double _y, double _z)
    {
      this->id = _agentId;
      this->x = _x;
      this->y = _y;
      this->z = _z;
    }

    public: int id;

    public: double x;

    public: double y;

    public: double z;
  };

  /// \brief InitMsg class contains information of init message.
  class InitMsg
  {
    /// \brief Constructor.
    /// \param[in] _agentId the socket ID.
    /// \param[in] _playerNum player number.
    /// \param[in] _teamName Team name.
    public: InitMsg(int _agentId, int _playerNum, std::string _teamName)
    {
      this->id = _agentId;
      this->playerNum = _playerNum;
      this->teamName = _teamName;
    }

    public: int id;

    public: int playerNum;

    public: std::string teamName;
  };

  /// \brief class instructor
  public: ActionMessageParser();

  /// \brief class destructor
  public: virtual ~ActionMessageParser();

  /// \brief Socket assigned in OnConnection()
  public: int socket;

  /// \brief global variables for determining new connections,
  /// Assigned in OnConnection()
  public: bool newConnectionDetected = false;

  /// \brief global variables for determining new disconnections
  /// assigned in OnDisconnection()
  public: bool newDisconnectionDetected = false;

  /// \brief Message received by socket
  public: std::stringstream message;

  /// \brief Data Structure used to store joints effector values
  public: std::map<std::string, double> jointParserMap;

  /// \brief Used to read incoming messages received by the socket.
  /// \param[in] _socket,
  /// return true when success or false otherwise.
  public: bool Parse(const int _socket);

  /// \brief Interface to access to scene information in data structure.
  /// \param[in] _id socket_Id,
  /// \param[out] _msg address message for the robot rsg model,
  /// \param[out] _robotType NAO robot types including {0 1 2 3}
  /// return true when the scene information exists in data structure or false otherwise.
  public: bool GetSceneInformation(const int _id, std::string &_msg,
      int &_robotType);

  /// \brief Interface for accessing to init information in data structure.
  /// \param[in] _id socket_Id,
  /// \param[out] _teamName name of the team,
  /// \param[out] _playerNumber number of the player,
  /// return true when the Inint information exists in data structure or false otherwise.
  public: bool GetInitInformation(const int _id, std::string &_teamName,
      int &_playerNumber);

  /// \brief Interface for accessing to information contained in Beam effector message.
  /// \param[in] _id socket_Id,
  /// \param[out] _x, _y, _z Position of beaming in the field
  /// return true when the Beam information exists in data structure or false otherwise.
  public: bool GetBeamInformation(const int _id, double &_x, double &_y,
      double &_z);

  /// \brief Used in server class constructor.
  public: void OnConnection(const int _socket);

  /// \brief Used in server class constructor.
  public: void OnDisconnection(const int );

  /// \brief Maximum size of each message received.
  private: static const int kBufferSize = 8192;

  private: int agentID;

  /// \brief data structure used for scene information.
  private: std::map<int, SceneMsg> sceneParserMap;

  /// \brief data structure used for init information.
  private: std::map<int, InitMsg> initParserMap;

  /// \brief data structure used for beam information.
  private: std::map<int, BeamMsg> beamParserMap;

  /// \brief Main procedure to extract information
  /// in pile of S-expressions.
  /// \param[in] _msg S-expressions message.
  public: void ParseMessage(const std::string &_msg);

  /// \brief Used to parse each S-expression message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseSexp(sexp_t *_exp);

  /// \brief Used to parse information in Scene effector message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseScene(sexp_t *_exp);

  /// \brief Used to parse information in Beam effector message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseBeam(sexp_t *_exp);

  /// \brief Used to parse information in Init message.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseInit(sexp_t *_exp);

  /// \brief Used to parse the joints effector value in S-expression messages.
  /// \param[in] _exp pointer to a S-expression.
  private: void ParseHingeJoint(sexp_t *exp);

};
#endif /* _GAZEBO_ROBOCUP3DS_ACTIONMESSAGEPARSER_HH_ */
