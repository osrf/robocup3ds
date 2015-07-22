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

#ifndef _GAZEBO_ROBOCUP3DS_EFFECTORPARSER_HH_
#define _GAZEBO_ROBOCUP3DS_EFFECTORPARSER_HH_

#include <cstring>
#include <map>
#include <memory>
#include <sstream>
#include <string>
#include <vector>
#include "robocup3ds/SocketParser.hh"
#include "../../lib/sexpLibrary/sexp.h"
#include "../../lib/sexpLibrary/sexp_ops.h"

/// \brief This is a Effector class. It implemented Parse() method
/// inherited from SocketParser class. Parse() method has been used
/// to receive messages from the socket.Retrieving effectors values
/// from received messages have been implemented using s-expression
/// library.
class EffectorParser: public SocketParser
{
  /// \brief SceneMsg class contains information of scene message.
  class SceneMsg
  {
    /// \brief Constructor.
    /// \param[in] _robotType robot type in Scene message.
    /// \param[in] _rsgAddress rsg file address.
    public: SceneMsg(const int _robotType, const std::string &_rsgAddress)
    {
      this->robotType = _robotType;
      this->rsgAddress = _rsgAddress;
    }
    /// \brief rsg file address
    public: std::string rsgAddress;
    /// \brief The robot type
    public: int robotType;
  };

  /// \brief BeamMsg class contains information of beam message.
  class BeamMsg
  {
    /// \brief Constructor.
    /// \param[in] _x, _y, _z Beam Position.
    public: BeamMsg(double _x, double _y, double _z)
    {
      this->x = _x;
      this->y = _y;
      this->z = _z;
    }
    /// \brief robot beam position position on x direction
    public: double x;
    /// \brief robot beam position position on y direction
    public: double y;
    /// \brief robot beam position position on z direction
    public: double z;
  };

  /// \brief InitMsg class contains information of Init message.
  class InitMsg
  {
    /// \brief Class constructor.
    /// \param[in] _playerNum Player number.
    /// \param[in] _teamName Team name.
    public: InitMsg(const int _playerNum, const std::string &_teamName)
    {
      this->playerNumber = _playerNum;
      this->teamName = _teamName;
    }
    /// \brief Player Number
    public: int playerNumber;
    /// \brief Team name
    public: std::string teamName;
  };

  /// \brief Class constructor.
  public: EffectorParser();

  /// \brief Class destructor.
  public: virtual ~EffectorParser() = default;

  /// \brief Used to read incoming message received by socket.
  /// \param[in] _socket, Socket of the client should be assigned by server.
  /// \return True when success or false otherwise.
  public: bool Parse(const int _socket);

  /// \brief Update obtained effectors values using s-expression library, and
  /// save them in empty data structures. It finds the agent belongs
  /// to the socket_ID then it updates agent's effectors in Agent Class.
  /// \param[in]  _socket. Socket assigned in Parse().
  public: void Update(int _socket);

  /// \brief Interface to access to scene information in data structure.
  /// \param[out] _rsgAddress RSG file address in computer belongs to the robot.
  /// \param[out] _robotType The robot type: robot types include {0 1 2 3}.
  /// \return True when the scene information exists in data structure or
  /// false otherwise.
  public: bool GetSceneInformation(std::string &_rsgAddress, int &_robotType);

  /// \brief Interface for accessing to Init information in data structure.
  /// \param[out] _teamName The team name.
  /// \param[out] _playerNumber The player number.
  /// \return True when the Init information exists in data structure or false
  /// otherwise.
  public: bool GetInitInformation(std::string &_teamName, int &_playerNumber);

  /// \brief Interface for accessing to Beam information.
  /// \param[out] _x Beaming position on x axis of the field
  /// \param[out] _y Beaming position on y axis of the field
  /// \param[out] _z Beaming position on z axis of the field
  /// \return True when the Beam information exists in data structure or false
  /// otherwise.
  public: bool GetBeamInformation(double &_x, double &_y, double &_z);

  /// \brief Interface for accessing to joint effectors.
  /// \param[in] _jointName, The joint name.
  /// Joint names and descriptions ordered by DoF number include:
  /// he1      1 Head Pitch
  /// he2      2 Head Yaw
  /// lle1     3 Left hip Pitch
  /// rle1     4 Right hip Pitch
  /// lle2     5 Left hip roll
  /// rle2     6 Right hip roll
  /// lle3     7 Left hip yaw
  /// rle3     8 Right hip yaw
  /// lle4     9 Left knee
  /// rle4     10 Right knee
  /// lle5     11 Left ankle pitch
  /// rle5     12 Right ankle pitch
  /// lle6     13 Left ankle roll
  /// rle6     14 Right ankle roll
  /// lae1     15 Left shoulder yaw
  /// rae1     16 Right shoulder yaw
  /// lae2     17 Left shoulder pitch
  /// rae2     18 Right shoulder pitch
  /// lae3     19 Left shoulder roll
  /// rae3     20 Right shoulder roll
  /// lae4     21 Left elbow
  /// rae4     22 Right elbow
  /// \param[out] _targetSpeed target angular speed as joint's effector value.
  /// \return True when the joint effector exists in data structure or false
  /// otherwise.
  public: bool GetJointEffector(const std::string &_jointName, double &_targetSpeed);

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

  /// \brief Socket assigned in OnConnection().
  public: int socketID;

  /// \brief Global variables for determining new connections,
  /// Assigned in OnConnection().
  public: bool newConnectionDetected = false;

  /// \brief global variables for determining new disconnections
  // assigned in OnDisconnection().
  public: bool newDisconnectionDetected = false;

  /// \brief Message received by socket and Parse().
  public: std::stringstream message;

  /// \brief Data Structure used to store Message received by sockets.
  /// Here, the key of the map is the Socket IDs which is assigned in
  /// OnConnection()
  public: std::map<int, InitMsg> socketIDAgentMap;

  /// \brief Data Structure used to store joints effector values.
  /// here, the key of the map is the Joints names which its descriptions
  /// can be found in the GetJointEffector() description.
  public: std::map<std::string, double> jointEffectors;

  /// \brief Maximum size of each message received.
  private: static const int kBufferSize = 8192;

  /// \brief data structure used for scene information.
  private: std::vector<SceneMsg> sceneEffectors;

  /// \brief data structure used for init information.
  private: std::vector<InitMsg> initEffectors;

  /// \brief data structure used for beam information.
  private: std::vector<BeamMsg> beamEffectors;

  /// \brief Protect concurrent access.
  private: mutable std::mutex mutex;

};
#endif /* _GAZEBO_ROBOCUP3DS_EFFECTORPARSER_HH_ */
