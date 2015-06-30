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

class ActionMessageParser
{
  public: enum Joints
  {
    he1,      // 0
    he2,      // 1
    lle1,     // 2
    rle1,     // 3
    lle2,     // 4
    rle2,     // 5
    lle3,     // 6
    rle3,     // 7
    lle4,     // 8
    rle4,     // 9
    lle5,     // 10
    rle5,     // 11
    lle6,     // 12
    rle6,     // 13
    lae1,     // 14
    rae1,     // 15
    lae2,     // 16
    rae2,     // 17
    lae3,     // 18
    rae3,     // 19
    lae4,     // 20
    rae4,     // 21
    NJOINTS,
  };

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

  public: static ActionMessageParser *GetUniqueInstance();

  public: virtual ~ActionMessageParser();

  public: void parseMessage(const std::string &_msg, int _agentID);

  public: bool getSceneInformation(const int _id, std::string &_msg,
      int &_robotType);

  public: bool getInitInformation(const int _id, std::string &_teamName,
      int &_playerNumber);

  public: std::map<int, double*> jointParserMap;

  public: std::map<int, SceneMsg> sceneParserMap;

  public: std::map<int, InitMsg> initParserMap;

  private: ActionMessageParser();

  private: int agentID;

  private: double jointsActions[NJOINTS];

  private: void parseSexp(sexp_t *exp);

  private: void parseScene(sexp_t *_exp);

  private: void parseInit(sexp_t *_exp);

  private: void parseHingeJoint(int jointID, sexp_t *exp);
};
#endif /* _GAZEBO_ROBOCUP3DS_ACTIONMESSAGEPARSER_HH_ */
