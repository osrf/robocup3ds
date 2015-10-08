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

#include <netinet/in.h>
#include <sys/socket.h>
#include <cerrno>
#include <cstdlib>
#include <iostream>
#include <ignition/math.hh>
#include <map>
#include <memory>
#include <mutex>
#include <string>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Effector.hh"
#include "robocup3ds/Util.hh"

using namespace ignition;
using namespace Util;

//////////////////////////////////////////////////
Effector::Effector(GameState *const _gameState):
  gameState(_gameState),
  currAgent(nullptr),
  currSocketId(-1)
{
}

//////////////////////////////////////////////////
Effector::~Effector()
{
}

//////////////////////////////////////////////////
bool Effector::Parse(int _socket)
{
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    // for some reason onConnection has not been called so return false
    if (this->socketIDMessageMap.find(_socket) == this->socketIDMessageMap.end()
        || this->socketIDMessageMap[_socket] == "__del__")
    {
      return false;
    }
  }

  bzero(this->buffer, sizeof(this->buffer));

  int bytesRead = 0;
  int totalBytes;

  // Used to read the size of the message in little-endian format
  int endianSize = recv(_socket, this->buffer, 4, 0);

  if (endianSize < 1)
  {
    return false;
  }

  // Calculate size of the s-expression messages
  uint32_t n;
  memcpy(&n, this->buffer, 4);
  totalBytes = ntohl(n);

  // Read the message using the size of actual s-expression message.
  while (bytesRead < totalBytes)
  {
    int result = recv(_socket, this->buffer + bytesRead,
                      totalBytes - bytesRead, 0);

    if (result < 1)
    {
      return false;
    }

    bytesRead += result;
  }

  std::string msg(buffer);

  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->socketIDMessageMap[_socket] == "__empty__")
  {
    this->socketIDMessageMap[_socket] = msg;
  }
  else
  {
    this->socketIDMessageMap[_socket]
      = this->socketIDMessageMap[_socket] + msg;
  }

  return true;
}

//////////////////////////////////////////////////
void Effector::ParseMessage(const std::string &_msg)
{
  sexp_t *exp;

  // Create a s-expression message using the received pile of s-expressions
  snprintf(this->sexpBuffer, Effector::kBufferSize, "(msg %s)", _msg.c_str());

  // use parse_sexp() from s-expression library
  exp = parse_sexp(this->sexpBuffer, _msg.length() + 6);

  if (!exp)
  {
    // std::cerr << "The message is empty" << std::endl;
    return;
  }
  else if (!exp->list)
  {
    // std::cerr << "The message is empty" << std::endl;
    return;
  }
  else if (!exp->list->next)
  {
    // std::cerr << "The message is empty" << std::endl;
    return;
  }

  sexp_t *ptr = exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      ParseSexp(ptr);
    }

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
      // std::cerr <<
      //         "Not in s-expression message format. the format: (value ....)"
      //           << std::endl;
      return;
    }
  }
  else
  {
    // std::cerr <<
    //           "Not an s-expression message. Not begin with a parenthesis"
    //           << std::endl;
    return;
  }

  // Decide based on the type of effector message
  if (!strcmp(v, "syn") && this->currAgent)
  {
    // std::cerr << "(syn) parsed" << std::endl;
    this->currAgent->isSynced = true;
  }
  else if (!strcmp(v, "beam"))
  {
    this->ParseBeam(_exp);
  }
  else if (!strcmp(v, "init"))
  {
    this->ParseInit(_exp);
  }
  else if (!strcmp(v, "scene"))
  {
    this->ParseScene(_exp);
  }
  else if (!strcmp(v, "say"))
  {
    this->ParseSay(_exp);
  }
  else if (this->currAgent
           && this->currAgent->bodyType->HingeJointEffectorMap().find(
             std::string(v))
           != this->currAgent->bodyType->HingeJointEffectorMap().end())
  {
    this->ParseHingeJoint(_exp);
  }
  else
  {
    // std::cerr << "Unknown Message : " << v << std::endl;
  }
}

//////////////////////////////////////////////////
void Effector::ParseHingeJoint(sexp_t *_exp)
{
  if (!this->currAgent)
  {
    return;
  }

  double angle;
  std::string jointName = _exp->list->val;
  if (_exp->list->next && S2D(_exp->list->next->val, angle))
  {
    this->currAgent->action.jointEffectors[jointName] = angle;
  }
}

////////////////////////////////////////////////
void Effector::ParseScene(sexp_t *_exp)
{
  // this is the case where we already have an agent in gameState,
  // then no need to use the Scene message
  if (this->currAgent)
  {
    return;
  }

  const std::string bodyType = _exp->list->next->val;
  if (this->gameState->agentBodyTypeMap.find(bodyType) !=
      this->gameState->agentBodyTypeMap.end())
  {
    this->socketIDbodyTypeMap[this->currSocketId] =
      this->gameState->agentBodyTypeMap.at(bodyType);
  }
  else
  {
    // use default body type if bodyType string is not recognized
    this->socketIDbodyTypeMap[this->currSocketId] =
      this->gameState->defaultBodyType;
  }
}

//////////////////////////////////////////////////
void Effector::ParseBeam(sexp_t *_exp)
{
  if (!this->currAgent)
  {
    return;
  }

  double x, y, yaw;
  if (_exp->list->next && _exp->list->next->next
      && _exp->list->next->next->next
      && S2D(_exp->list->next->val, x)
      && S2D(_exp->list->next->next->val, y)
      && S2D(_exp->list->next->next->next->val, yaw))
  {
    this->gameState->BeamAgent(this->currAgent->uNum,
                               this->currAgent->team->name, x, y, yaw);
    // gzmsg << "beamed agent (" << x << "," << y << "," << yaw
    //       << "): " << this->currAgent->GetName() << std::endl;
  }
}

//////////////////////////////////////////////////
void Effector::ParseSay(sexp_t *_exp)
{
  if (!this->currAgent)
  {
    return;
  }
  // If there is already a valid message, dont overwrite current message with
  // new one
  if (this->currAgent->team->say.isValid)
  {
    return;
  }

  // Accept the say message that does not contains the white space
  // characters and S-expression phrases
  if (_exp->list->next && !_exp->list->next->next)
  {
    const std::string message = _exp->list->next->val;
    const int size = message.length();

    // Accept only say message less than 20 characters
    if (size > 20)
    {
      return;
    }

    // Accept only printing characters
    for (int i = 0; i < size; ++i)
    {
      const int asciiVal = static_cast<int>(message[i]);
      if ( asciiVal <= 32 || asciiVal >= 127 )
      {
        return;
      }
    }

    // Update the say message container
    this->currAgent->team->say.isValid = true;
    this->currAgent->team->say.msg = message;
    this->currAgent->team->say.agentId = this->currAgent->GetAgentID();
    this->currAgent->team->say.pos = this->currAgent->pos;
  }
}

//////////////////////////////////////////////////
void Effector::ParseInit(sexp_t *_exp)
{
  // this is the case where we already have an agent in gameState,
  // then no need for init
  if (this->currAgent
      || (this->socketIDbodyTypeMap.find(this->currSocketId)
          == this->socketIDbodyTypeMap.end()))
  {
    return;
  }

  int playerNum = -1;
  std::string teamName = "";

  sexp_t *ptr = _exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "unum") && ptr->list->next)
      {
        double temp;
        if (S2D(ptr->list->next->val, temp))
        {
          playerNum = static_cast<int>(temp);
        }
      }
      else if (!strcmp(ptr->list->val, "teamname") && ptr->list->next)
      {
        teamName = ptr->list->next->val;
      }
    }
    ptr = ptr->next;
  }

  this->currAgent = this->gameState->AddAgent(
                      playerNum, teamName,
                      this->socketIDbodyTypeMap.at(this->currSocketId),
                      this->currSocketId);
  this->socketIDbodyTypeMap.erase(this->currSocketId);

  if (this->currAgent)
  {
    this->agentsToAdd.push_back(this->currAgent);
    gzmsg << "(" << this->gameState->GetGameTime() <<
          ") agent added to game state: " << this->currAgent->GetName()
          << std::endl;
  }
  else
  {
    this->socketsToDisconnect.push_back(this->currSocketId);
    gzmsg << "(" << this->gameState->GetGameTime() <<
          ") failed to add agent to game state: " <<
          Agent::GetName(playerNum, teamName) << std::endl;
  }
}

//////////////////////////////////////////////////
void Effector::OnConnection(const int _socket)
{
  // std::cerr << "socket " << _socket << " connected!" << std::endl;
  std::lock_guard<std::mutex> lock(this->mutex);
  this->socketIDMessageMap[_socket] = "__empty__";
}

//////////////////////////////////////////////////
void Effector::OnDisconnection(const int _socket)
{
  // std::cerr << "socket " << _socket << " disconnected!" << std::endl;
  std::lock_guard<std::mutex> lock(this->mutex);
  if (this->socketIDMessageMap.find(_socket) !=
      this->socketIDMessageMap.end())
  {
    this->socketIDMessageMap[_socket] = "__del__";
  }
}

//////////////////////////////////////////////////
void Effector::Update()
{
  // clear data structures
  this->agentsToAdd.clear();
  this->agentsToRemove.clear();
  this->socketsToDisconnect.clear();

  std::map <int, Agent *> socketIdAgentMap;

  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      socketIdAgentMap[agent.socketID] = &agent;
    }
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  // Update Effectors using message received by Parse()
  for (auto kv = this->socketIDMessageMap.begin();
       kv != this->socketIDMessageMap.end();)
  {
    this->currSocketId = kv->first;
    this->currAgent = nullptr;
    if (socketIdAgentMap.find(this->currSocketId) != socketIdAgentMap.end())
    {
      this->currAgent = socketIdAgentMap[this->currSocketId];
    }

    if (kv->second == "__del__")
    {
      if (this->currAgent && this->gameState->RemoveAgent(this->currAgent->uNum,
          this->currAgent->team->name))
      {
        this->agentsToRemove.push_back(this->currAgent->GetName());
      }
      this->socketIDMessageMap.erase(kv++);
    }
    else if (kv->second == "__empty__")
    {
      ++kv;
    }
    else
    {
      this->ParseMessage(kv->second);
      this->socketIDMessageMap[kv->first] = "__empty__";
      ++kv;
    }
  }

  this->currSocketId = -1;
  this->currAgent = NULL;
}

//////////////////////////////////////////////////
MonitorEffector::MonitorEffector(GameState *const _gameState):
  Effector(_gameState)
{}

//////////////////////////////////////////////////
void MonitorEffector::Update()
{
  this->agentsToRemove.clear();
  // this->agentsToAdd.clear();
  // this->socketsToDisconnect.clear();

  std::lock_guard<std::mutex> lock(this->mutex);

  // Update Effectors using message received by Parse()
  for (auto kv = this->socketIDMessageMap.begin();
       kv != this->socketIDMessageMap.end();)
  {
    if (kv->second == "__del__")
    {
      this->socketIDMessageMap.erase(kv++);
    }
    else if (kv->second == "__empty__")
    {
      ++kv;
    }
    else
    {
      this->ParseMessage(kv->second);
      this->socketIDMessageMap[kv->first] = "__empty__";
      ++kv;
    }
  }
}

//////////////////////////////////////////////////
void MonitorEffector::ParseSexp(sexp_t *_exp)
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
      // std::cerr <<
      //          "Not in s-expression message format. the format: (value ....)"
      //           << std::endl;
      return;
    }
  }
  else
  {
    // std::cerr <<
    //           "Not an s-expression message. Not begin with a parenthesis"
    //           << std::endl;
    return;
  }

  // Decide based on the type of effector message
  if (!strcmp(v, "agent"))
  {
    this->ParseMoveAgent(_exp);
  }
  else if (!strcmp(v, "ball"))
  {
    this->ParseMoveBall(_exp);
  }
  else if (!strcmp(v, "playMode"))
  {
    this->ParsePlayMode(_exp);
  }
  else if (!strcmp(v, "kill"))
  {
    this->ParseRemoveAgent(_exp);
  }
  else
  {
    // std::cerr << "Unknown Message : " << v << std::endl;
  }
}

//////////////////////////////////////////////////
void MonitorEffector::ParseMoveAgent(sexp_t *_exp)
{
  int uNum = -1;
  std::string teamSide = "";
  double x, y, z, yaw;
  x = y = z = yaw = -1;
  bool pMove = false;
  bool pUNum = false;
  bool pTeamS = false;
  bool pDouble = false;

  sexp_t *ptr = _exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "unum") && ptr->list->next)
      {
        double temp;
        if (S2D(ptr->list->next->val, temp))
        {
          uNum = static_cast<int>(temp);
        }
        pUNum = true;
      }
      else if (!strcmp(ptr->list->val, "team") && ptr->list->next)
      {
        teamSide = ptr->list->next->val;
        pTeamS = true;
      }
      else if (!strcmp(ptr->list->val, "pos") && ptr->list->next
               && ptr->list->next->next && ptr->list->next->next->next)
      {
        if (S2D(ptr->list->next->val, x)
            && S2D(ptr->list->next->next->val, y)
            && S2D(ptr->list->next->next->next->val, z))
        {
          pMove = false;
          pDouble = true;
        }
        else
        {
          pDouble = false;
        }
      }
      else if (!strcmp(ptr->list->val, "move") && ptr->list->next
               && ptr->list->next->next && ptr->list->next->next->next
               && ptr->list->next->next->next->next)
      {
        if (S2D(ptr->list->next->val, x)
            && S2D(ptr->list->next->next->val, y)
            && S2D(ptr->list->next->next->next->val, z)
            && S2D(ptr->list->next->next->next->next->val, yaw))
        {
          pMove = true;
          pDouble = true;
        }
        else
        {
          pDouble = false;
        }
      }
    }
    ptr = ptr->next;
  }

  if (!pUNum || !pTeamS || !pDouble)
  {
    return;
  }

  if (uNum < 0 || uNum > 11)
  {
    return;
  }

  Team::Side side = Team::GetSideAsEnum(teamSide);
  if (side == Team::Side::NEITHER)
  {
    return;
  }

  auto newPos = math::Vector3<double>(x, y, z);
  for (const auto &team : this->gameState->teams)
  {
    if (team->side != side)
    {
      continue;
    }
    for (auto &agent : team->members)
    {
      if (agent.uNum != uNum)
      {
        continue;
      }
      if (pMove)
      {
        auto newRot = math::Quaternion<double>(0, 0, yaw);
        this->gameState->MoveAgent(agent, newPos, newRot);
      }
      else
      {
        this->gameState->MoveAgent(agent, newPos);
      }
    }
  }
}

//////////////////////////////////////////////////
void MonitorEffector::ParseMoveBall(sexp_t *_exp)
{
  double x, y, z;
  double u, v, w;
  sexp_t *ptr = _exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "pos") && ptr->list->next
          && ptr->list->next->next && ptr->list->next->next->next
          && S2D(ptr->list->next->val, x)
          && S2D(ptr->list->next->next->val, y)
          && S2D(ptr->list->next->next->next->val, z))
      {
        this->gameState->MoveBall(math::Vector3<double>(x, y, z));
      }
      else if (!strcmp(ptr->list->val, "vel") && ptr->list->next
               && ptr->list->next->next && ptr->list->next->next->next
               && S2D(ptr->list->next->val, u)
               && S2D(ptr->list->next->next->val, v)
               && S2D(ptr->list->next->next->next->val, w))
      {
        this->gameState->SetBallVel(math::Vector3<double>(u, v, w));
      }
    }
    ptr = ptr->next;
  }
}

//////////////////////////////////////////////////
void MonitorEffector::ParsePlayMode(sexp_t *_exp)
{
  if (_exp->list->next)
  {
    std::string playMode = _exp->list->next->val;
    if (this->gameState->playModeNameMap.find(playMode)
        != this->gameState->playModeNameMap.end())
    {
      this->gameState->SetCurrent(this->gameState->playModeNameMap[playMode]);
    }
  }
}

//////////////////////////////////////////////////
void MonitorEffector::ParseRemoveAgent(sexp_t *_exp)
{
  int uNum = -1;
  std::string teamSide = "";
  bool pUNum = false;
  bool pTeamS = false;

  sexp_t *ptr = _exp->list->next;
  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "unum") && ptr->list->next)
      {
        double temp;
        if (S2D(ptr->list->next->val, temp))
        {
          uNum = static_cast<int>(temp);
        }
        pUNum = true;
      }
      else if (!strcmp(ptr->list->val, "team") && ptr->list->next)
      {
        teamSide = ptr->list->next->val;
        pTeamS = true;
      }
    }
    ptr = ptr->next;
  }

  if (!pUNum || !pTeamS)
  {
    return;
  }

  if (uNum < 0 || uNum > 11)
  {
    return;
  }

  Team::Side side = Team::GetSideAsEnum(teamSide);
  if (side == Team::Side::NEITHER)
  {
    return;
  }

  std::string teamName;
  for (const auto &team : this->gameState->teams)
  {
    if (team->side == side)
    {
      teamName = team->name;
    }
  }

  if (this->gameState->RemoveAgent(uNum, teamName))
  {
    this->agentsToRemove.push_back(Agent::GetName(uNum, teamName));
  }
}
