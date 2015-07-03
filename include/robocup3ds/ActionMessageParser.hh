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

  class SceneMsg
  {
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

  class BeamMsg
  {
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

  class InitMsg
  {
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

  private: std::map<int, SceneMsg> sceneParserMap;

  private: std::map<int, InitMsg> initParserMap;

  private: std::map<int, BeamMsg> beamParserMap;

  private: int agentID;

  private: void parseSexp(sexp_t *exp);

  private: void parseScene(sexp_t *_exp);

  private: void parseBeam(sexp_t *_exp);

  private: void parseInit(sexp_t *_exp);

  private: void parseHingeJoint(sexp_t *exp);

  public: std::stringstream message;

  public: ActionMessageParser();

  public: bool Parse(const int _socket);

  public: void parseMessage(const std::string &_msg);

  public: bool getSceneInformation(const int _id, std::string &_msg,
      int &_robotType);

  public: bool getInitInformation(const int _id, std::string &_teamName,
      int &_playerNumber);

  public: bool getBeamInformation(const int _id, double &_x, double &_y,
      double &_z);

  public: std::map<std::string, double> jointParserMap;

  public: void OnConnection(const int _socket)
  {
    this->socket = _socket;
  }

  public: void OnDisconnection(const int /*_socket*/)
  {
  }

  private: static const int kBufferSize = 8192;

  private: int socket;

  public: virtual ~ActionMessageParser();
};
#endif /* _GAZEBO_ROBOCUP3DS_ACTIONMESSAGEPARSER_HH_ */
