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
#include <iostream>
#include <map>
#include <mutex>
#include <string>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Effector.hh"

const int Effector::kBufferSize = 16384;

//////////////////////////////////////////////////
Effector::Effector(GameState *const _gameState):
  currAgent(NULL),
  gameState(_gameState)
{
  // Initialize global variables
  this->buffer = new char[Effector::kBufferSize];
  this->sexpBuffer = new char[Effector::kBufferSize];
}

Effector::~Effector()
{
  delete[] this->buffer;
  delete[] this->sexpBuffer;
}

//////////////////////////////////////////////////
bool Effector::Parse(int _socket)
{
  // for some reason onConnection has not been called so return false
  if (this->socketIDMessageMap.find(_socket) == this->socketIDMessageMap.end())
  {
    return false;
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

  // Avoiding race condition
  std::lock_guard<std::mutex> lock(this->mutex);
  this->socketIDMessageMap[_socket] = std::string(buffer);
  return true;
}

//////////////////////////////////////////////////
void Effector::ParseMessage(const std::string &_msg)
{
  sexp_t *exp;

  // Create a s-expression message using the received pile of s-expressions
  snprintf(this->sexpBuffer, Effector::kBufferSize, "(msg %s)",
           _msg.c_str());

  // use parse_sexp() from s-expression library
  exp = parse_sexp(this->sexpBuffer, _msg.length() + 6);

  if (!exp)
  {
    std::cerr << "The message is empty" << std::endl;
    return;
  }
  else if (!exp->list)
  {
    std::cerr << "The message is empty" << std::endl;
    return;
  }
  else if (!exp->list->next)
  {
    std::cerr << "The message is empty" << std::endl;
    return;
  }

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
    std::cerr <<
              "Not an s-expression message. Not begin with a parenthesis"
              << std::endl;
    return;
  }

  // Decide based on the type of effector message
  if (!strcmp(v, "scene"))
  {
    // this->ParseScene(_exp);
  }
  else if (!strcmp(v, "beam"))
  {
    this->ParseBeam(_exp);
  }
  else if (!strcmp(v, "init"))
  {
    this->ParseInit(_exp);
  }
  else if (NaoRobot::hingeJointEffectorMap.find(std::string(v))
           != NaoRobot::hingeJointEffectorMap.end())
  {
    ParseHingeJoint(_exp);
  }
  else
  {
    std::cerr << "Unknown Message : " << v << std::endl;
  }
}

//////////////////////////////////////////////////
void Effector::ParseHingeJoint(sexp_t *_exp)
{
  if (!this->currAgent)
  {
    return;
  }

  std::string jointName = _exp->list->val;
  double angle = atof(_exp->list->next->val);

  this->currAgent->action.jointEffectors[jointName] = angle;
}

//////////////////////////////////////////////////
// void Effector::ParseScene(sexp_t *_exp)
// {
// int type = 0;
// std::string address;

// address = _exp->list->next->val;
// type = atof(_exp->list->next->next->val);

// this->sceneEffectors.push_back(SceneMsg(type, address));
// }

//////////////////////////////////////////////////
void Effector::ParseBeam(sexp_t *_exp)
{
  if (!this->currAgent)
  {
    return;
  }

  double x, y, yaw = 0;

  x = atof(_exp->list->next->val);

  y = atof(_exp->list->next->next->val);

  yaw = atof(_exp->list->next->next->next->val);

  this->gameState->BeamAgent(this->currAgent->uNum,
                             this->currAgent->team->name, x, y, yaw);
}

//////////////////////////////////////////////////
void Effector::ParseInit(sexp_t *_exp)
{
  // this is the case where we already have an agent in gameState,
  // then no need for init
  if (this->currAgent)
  {
    return;
  }

  int playerNum = 0;

  std::string teamName = "";

  sexp_t *ptr = _exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
    {
      if (!strcmp(ptr->list->val, "unum"))
      {
        playerNum = atof(ptr->list->next->val);
      }
      if (!strcmp(ptr->list->val, "teamname"))
      {
        teamName = ptr->list->next->val;
      }
    }
    ptr = ptr->next;
  }

  if (this->gameState->AddAgent(playerNum, teamName, this->currSocketId))
  {
    AgentId agentId(playerNum, teamName);
    this->agentsToAdd.push_back(agentId);
  }
}

//////////////////////////////////////////////////
void Effector::OnConnection(const int _socket)
{
  std::lock_guard<std::mutex> lock(this->mutex);
  this->socketIDMessageMap[_socket] = "__init__";
}

//////////////////////////////////////////////////
void Effector::OnDisconnection(const int _socket)
{
  if (this->socketIDMessageMap.find(_socket) !=
      this->socketIDMessageMap.end())
  {
    std::lock_guard<std::mutex> lock(this->mutex);
    this->socketIDMessageMap[_socket] = "__del__";
  }
}

//////////////////////////////////////////////////
void Effector::Update()
{
  // clear data structures
  this->agentsToAdd.clear();
  this->agentsToRemove.clear();

  // Update Effectors using message received by Parse()
  std::lock_guard<std::mutex> lock(this->mutex);
  for (auto kv = this->socketIDMessageMap.begin();
       kv != this->socketIDMessageMap.end();)
  {
    this->currAgent = NULL;
    this->currSocketId = kv->first;
    for (const auto &team : this->gameState->teams)
    {
      for (auto &agent : team->members)
      {
        if (this->currSocketId == agent.socketID)
        {
          this->currAgent = &agent;
        }
      }
    }

    if (kv->second == "__del__")
    {
      if (this->currAgent && this->gameState->RemoveAgent(this->currAgent->uNum,
          this->currAgent->team->name))
      {
        this->agentsToRemove.push_back(this->currAgent->GetAgentID());
      }
      this->socketIDMessageMap.erase(kv++);
    }
    else
    {
      ParseMessage(kv->second);
      kv++;
    }
  }
}
