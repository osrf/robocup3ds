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

#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include <map>
#include <string>

#include "../../lib/sexpLibrary/sexp.h"
#include "../../lib/sexpLibrary/sexp_ops.h"

#include "robocup3ds/Effectors.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Nao.hh"

//////////////////////////////////////////////////
Effector::Effector(GameState *const _gameState):
  newConnectionDetected(false),
  newDisconnectionDetected(false),
  gameState(_gameState)
{
  // Initialize global variables
  this->socketID = 0;
  this->message.str("");
}

//////////////////////////////////////////////////
bool Effector::Parse(int _socket)
{
  char buffer[this->kBufferSize];

  bzero(buffer, sizeof(buffer));

  int bytesRead = 0;

  int totalBytes;

  // Used to read the size of the message in little-endian format
  int endianSize = recv(_socket, buffer, 4, 0);

  if (endianSize < 1)
  {
    return false;
  }

  // Calculate size of the s-expression messages
  uint32_t n;
  memcpy(&n, buffer, 4);
  totalBytes = ntohl(n);

  // Read the message using the size of actual s-expression message.
  while (bytesRead < totalBytes)
  {
    int result = recv(_socket, buffer + bytesRead, totalBytes - bytesRead, 0);

    if (result < 1)
    {
      return false;
    }

    bytesRead += result;
  }

  // Clear the message
  this->message.str("");
  // Store the received s-expression messages
  this->message << std::string(buffer);

  return true;
}

//////////////////////////////////////////////////
void Effector::ParseMessage(const std::string &_msg)
{
  char linebuf[36000];
  sexp_t *exp;

  std::stringstream s_expression;

  // Create a s-expression message using the received pile of s-expressions
  s_expression << "(msg " << _msg << ")";

  snprintf(linebuf, sizeof(linebuf), "%s", s_expression.str().c_str());

  // use parse_sexp() from s-expression library
  exp = parse_sexp(linebuf, 36000);

  if (exp == NULL)
  { return; }
  else if (exp->list == NULL)
  { return; }
  else if (exp->list->next == NULL)
  { return; }
  sexp_t *ptr = exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    { ParseSexp(ptr); }

    ptr = ptr->next;
  }

  destroy_sexp(exp);
}

//////////////////////////////////////////////////
void Effector::ParseSexp(sexp_t *_exp)
{
  // Extract the first element of a s-expression
  char *v;
  if (_exp->ty == SEXP_LIST)
  {
    if (_exp->list->ty == SEXP_VALUE)
    {
      v = _exp->list->val;
    }
    else
    {
      std::cerr <<
                "Not in s-expression message format. the format: (value ....)"
                << std::endl;
      return;
    }
  }
  else
  {
    std::cerr << "Not an s-expression message. Not begin with a parenthesis" <<
              std::endl;
    return;
  }


  // Decide based on the type of effector message
  if (!strcmp(v, "scene"))
  {
    this->ParseScene(_exp);
  }
  else if (!strcmp(v, "beam"))
  {
    this->ParseBeam(_exp);
  }
  else if (!strcmp(v, "init"))
  {
    this->ParseInit(_exp);
  }
  else if (NaoRobot::hingeJointEffectorMap.find(std::string(v)) !=
           NaoRobot::hingeJointEffectorMap.end())
  {
    this->ParseHingeJoint(_exp);
  }
  else
  {
    std::cerr << "Unknown Message : " << v << std::endl;
  }
}

//////////////////////////////////////////////////
void Effector::ParseHingeJoint(sexp_t *_exp)
{
  std::string name;
  double effector;

  name = _exp->list->val;
  effector = atof(_exp->list->next->val);

  this->jointEffectors.insert(
    std::map <std::string, double> ::value_type(name, effector));
}

//////////////////////////////////////////////////
void Effector::ParseScene(sexp_t *_exp)
{
  int type = 0;
  std::string address;

  address = _exp->list->next->val;
  type = atof(_exp->list->next->next->val);

  this->sceneEffectors.push_back(SceneMsg(type, address));
}

//////////////////////////////////////////////////
void Effector::ParseBeam(sexp_t *_exp)
{
  double x, y, z = 0;

  x = atof(_exp->list->next->val);

  y = atof(_exp->list->next->next->val);

  z = atof(_exp->list->next->next->next->val);

  this->beamEffectors.push_back(BeamMsg(x, y, z));
}

//////////////////////////////////////////////////
void Effector::ParseInit(sexp_t *_exp)
{
  int uNum = 0;

  std::string teamName = "";

  sexp_t *ptr = _exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "unum"))
      {
        uNum = atof(ptr->list->next->val);
      }
      if (!strcmp(ptr->list->val, "teamname"))
      {
        teamName = ptr->list->next->val;
      }
    }
    ptr = ptr->next;
  }

  this->agentsToAdd.push_back(InitMsg(uNum, teamName));
}

//////////////////////////////////////////////////
void Effector::OnConnection(const int _socket)
{
  this->socketID = _socket;
  this->newConnectionDetected = true;
}

//////////////////////////////////////////////////
void Effector::OnDisconnection(const int /*_socket*/)
{
  this->newDisconnectionDetected = true;
}

//////////////////////////////////////////////////
void Effector::Update()
{
  // clear data structures
  this->beamEffectors.clear();
  this->agentsToAdd.clear();
  this->sceneEffectors.clear();
  this->jointEffectors.clear();

  // Update Effectors using message received by Parse()
  ParseMessage(this->message.str());
}

//////////////////////////////////////////////////
bool Effector::GetSceneInformation(std::string &_msg, int &_robotType)
{
  if (!this->sceneEffectors.empty())
  {
    _msg = this->sceneEffectors.front().rsgAddress;
    _robotType = this->sceneEffectors.front().robotType;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool Effector::GetInitInformation(std::string &_teamName,
                                  int &_uNum)
{
  if (!this->agentsToAdd.empty())
  {
    _teamName = this->agentsToAdd.front().teamName;
    _uNum = this->agentsToAdd.front().uNum;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool Effector::GetBeamInformation(double &_x, double &_y, double &_z)
{
  if (!this->agentsToAdd.empty())
  {
    _x = this->beamEffectors.front().x;
    _y = this->beamEffectors.front().y;
    _z = this->beamEffectors.front().z;

    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool Effector::GetJointEffector(
  std::string _jointName, double &_targetSpeed)
{
  std::map<std::string, double>::const_iterator it =
    this->jointEffectors.find(_jointName);

  if (it == this->jointEffectors.end()) { return false; }

  _targetSpeed = this->jointEffectors.find(_jointName)->second;

  return true;
}

//////////////////////////////////////////////////
Effector::~Effector()
{
}
