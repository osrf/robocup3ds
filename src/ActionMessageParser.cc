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

#include "robocup3ds/ActionMessageParser.hh"
#include <sys/socket.h>

ActionMessageParser::ActionMessageParser()
{

}

bool ActionMessageParser::Parse(int _socket)
{
  char buffer[8092];
  bzero(buffer, sizeof(buffer));
  int bytesRead = 0;
  int totalBytes = 1000;

  while (bytesRead < totalBytes)
  {
    int result = recv(_socket, buffer + bytesRead, totalBytes - bytesRead, 0);
    if (result < 1)
    {
      return false;
    }

    bytesRead += result;
  }

  parseMessage(std::string(buffer));

  return true;
}


void ActionMessageParser::parseMessage(const std::string &_msg)
{
  char linebuf[BUFSIZ+16000];
  sexp_t *exp;

//  std::stringstream message;

  this->message << "(msg " << _msg << ")";

  strcpy(linebuf, message.str().c_str());

  exp = parse_sexp(linebuf, BUFSIZ+16000);

  if (exp == NULL)
    return;
  else if (exp->list == NULL)
    return;
  else if (exp->list->next == NULL)
    return;

  sexp_t* ptr = exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
      parseSexp(ptr);

    ptr = ptr->next;
  }

  destroy_sexp(exp);
}

void ActionMessageParser::parseSexp(sexp_t *_exp)
{
  char *v;

  if (_exp->ty == SEXP_LIST)
  {
    if (_exp->list->ty == SEXP_VALUE)
      v = _exp->list->val;
    else
      return;
  }
  else
    return;

  if (!strcmp(v, "scene"))
  {
    parseScene(_exp);
  }
  else if (!strcmp(v, "beam"))
  {
    parseBeam(_exp);
  }
  else if (!strcmp(v, "init"))
  {
    parseInit(_exp);
  }
  else if (!strcmp(v, "he1"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "he2"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle1"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle1"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle2"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle2"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle3"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle3"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle4"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle4"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle5"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle5"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle6"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle6"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae1"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae1"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae2"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae2"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae3"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae3"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae4"))
  {
    parseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae4"))
  {
    parseHingeJoint(_exp);
  }
  else
  {
    fprintf(stderr, "EVAL: Unknown = %s\n", v);
  }
}

void ActionMessageParser::parseHingeJoint(sexp_t *_exp)
{
  std::string name;
  double effector;
  name =_exp->list->val;
  effector = atof(_exp->list->next->val);
  this->jointParserMap.insert(std::map <std::string, double> ::value_type(name, effector));
}

void ActionMessageParser::parseScene(sexp_t *_exp)
{
  int type = 0;

  std::string address;

  address =_exp->list->next->val;

  type = atof(_exp->list->next->next->val);

  this->sceneParserMap.insert(std::map<int, SceneMsg >::value_type(this->agentID
      , SceneMsg(this->agentID, type, address)));
}

void ActionMessageParser::parseBeam(sexp_t *_exp)
{
  double x,y,z = 0;

  x = atof(_exp->list->next->val);

  y = atof(_exp->list->next->next->val);

  z = atof(_exp->list->next->next->next->val);

  this->beamParserMap.insert(std::map<int, BeamMsg >::value_type(this->agentID
      , BeamMsg(this->agentID, x, y, z)));
}

void ActionMessageParser::parseInit(sexp_t *_exp)
{
  int playerNum = 0;

  std::string teamName= "";

  sexp_t* ptr = _exp->list->next;

  while (ptr != NULL) {
    if (ptr->ty == SEXP_LIST) {
      if (!strcmp(ptr->list->val, "unum"))
      {
        playerNum = atof(ptr->list->next->val);
      }
      if (!strcmp(ptr->list->val, "teamname"))
      {
        teamName= ptr->list->next->val;
      }
    }
    ptr = ptr->next;
  }

  this->initParserMap.insert(std::map<int, InitMsg >::value_type(this->agentID
      , InitMsg(this->agentID, playerNum, teamName)));
}

bool ActionMessageParser::getSceneInformation(const int _id, std::string &_msg, int &_robotType)
{
  for (auto ob = this->sceneParserMap.begin(); ob != this->sceneParserMap.end(); ++ob)
  {
    if (ob->first == _id)
    {
      _msg= ob->second.rsgAddress;
      _robotType= ob->second.robotType;
      return true;
    }
  }
  return false;
}

bool ActionMessageParser::getInitInformation(const int _id, std::string &_teamName,
    int &_playerNumber)
{
  for (auto ob = this->initParserMap.begin(); ob != this->initParserMap.end(); ++ob)
  {
    if (ob->first == _id)
    {
      _teamName= ob->second.teamName;
      _playerNumber= ob->second.playerNum;
      return true;
    }
  }

  return false;
}

bool ActionMessageParser::getBeamInformation(const int _id, double &_x, double &_y,
    double &_z)
{
  for (auto ob = this->beamParserMap.begin(); ob != this->beamParserMap.end(); ++ob)
  {
    if (ob->first == _id)
    {
      _x = ob->second.x;
      _y = ob->second.y;
      _z = ob->second.z;
      return true;
    }
  }
  return false;
}

ActionMessageParser::~ActionMessageParser()
{
}
