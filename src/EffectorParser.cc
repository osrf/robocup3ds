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

#include <iostream>
#include <netinet/in.h>
#include <mutex>
#include <sys/socket.h>
#include "robocup3ds/EffectorParser.hh"

//////////////////////////////////////////////////
EffectorParser::EffectorParser()
{
  //Initialize global variables
  this->socketID = 0;
  this->message.str("");
}

//////////////////////////////////////////////////
bool EffectorParser::Parse(int _socket)
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

  //Avoiding race condition
  std::lock_guard<std::mutex> lock(this->mutex);

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

  // Perform information retrieving procedure and update data structures.
  // Also find the Agent belong to the socket in order to update its containers.
  Update(_socket);

  return true;
}

//////////////////////////////////////////////////
void EffectorParser::ParseMessage(const std::string &_msg)
{
  char linebuf[36000];
  sexp_t *exp;

  std::stringstream s_expression;

  // Create a s-expression message using the received pile of s-expressions
  s_expression << "(msg " << _msg << ")";

  strcpy(linebuf, s_expression.str().c_str());

  // use parse_sexp() from s-expression library
  exp = parse_sexp(linebuf, 36000);

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

  sexp_t* ptr = exp->list->next;

  while (ptr != NULL)
  {
    if (ptr->ty == SEXP_LIST)
      ParseSexp(ptr);

    ptr = ptr->next;
  }

  destroy_sexp(exp);
}

//////////////////////////////////////////////////
void EffectorParser::ParseSexp(sexp_t *_exp)
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
  else if (!strcmp(v, "he1") || !strcmp(v, "he2") || !strcmp(v, "lle1")
    || !strcmp(v, "rle1") || !strcmp(v, "lle2") || !strcmp(v, "rle2")
    || !strcmp(v, "lle3") || !strcmp(v, "rle3") || !strcmp(v, "lle4")
    || !strcmp(v, "rle4") || !strcmp(v, "lle5") || !strcmp(v, "rle5")
    || !strcmp(v, "lle6") || !strcmp(v, "rle6") || !strcmp(v, "lae1")
    || !strcmp(v, "rae1") || !strcmp(v, "lae2") || !strcmp(v, "rae2")
    || !strcmp(v, "lae3") || !strcmp(v, "rae3") || !strcmp(v, "lae4")
    || !strcmp(v, "rae4"))
  {
    ParseHingeJoint(_exp);
  }
  else
  {
    std::cerr << "Unknown Message : " <<v<< std::endl;
  }
}

//////////////////////////////////////////////////
void EffectorParser::ParseHingeJoint(sexp_t *_exp)
{
  std::string name;
  double effector;

  name =_exp->list->val;
  effector = atof(_exp->list->next->val);

  this->jointEffectors.insert(
      std::map <std::string, double> ::value_type(name, effector));
}

//////////////////////////////////////////////////
void EffectorParser::ParseScene(sexp_t *_exp)
{
  int type = 0;
  std::string address;

  address =_exp->list->next->val;
  type = atof(_exp->list->next->next->val);

  this->sceneEffectors.push_back(SceneMsg(type, address));
}

//////////////////////////////////////////////////
void EffectorParser::ParseBeam(sexp_t *_exp)
{
  double x, y, z = 0;

  x = atof(_exp->list->next->val);

  y = atof(_exp->list->next->next->val);

  z = atof(_exp->list->next->next->next->val);

  this->beamEffectors.push_back(BeamMsg(x, y, z));
}

//////////////////////////////////////////////////
void EffectorParser::ParseInit(sexp_t *_exp)
{
  int playerNum = 0;

  std::string teamName= "";

  sexp_t* ptr = _exp->list->next;

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
        teamName= ptr->list->next->val;
      }
    }
    ptr = ptr->next;
  }

  this->initEffectors.push_back(InitMsg(playerNum, teamName));
}

//////////////////////////////////////////////////
void EffectorParser::OnConnection(const int _socket)
{
  this->socketID = _socket;
  this->newConnectionDetected = true;
}

//////////////////////////////////////////////////
void EffectorParser::OnDisconnection(const int _socket)
{
  // If the socket belongs to an initialized agent, remove that agent
  if(this->socketIDAgentMap.find(_socket) != this->socketIDAgentMap.end())
  {
    /* uncomment after integration
      gameState->RemoveAgent(
        this->socketIDAgentMap.find(_socket)->second.playerNumber,
        this->socketIDAgentMap.find(_socket)->second.teamName);
    */
  }

  this->newDisconnectionDetected = true;
}

//////////////////////////////////////////////////
void EffectorParser::Update(int _socket)
{
  // clear data structures
  this->beamEffectors.clear();
  this->initEffectors.clear();
  this->sceneEffectors.clear();
  this->jointEffectors.clear();

  // Update Effectors using message received by Parse()
  ParseMessage(this->message.str());

  int playerNumber;
  std::string teamName;

  // If init message received, and the agent belongs to that message had not been
  // initialized then initialize that agent
  if ( this->GetInitInformation(teamName, playerNumber)
      && socketIDAgentMap.find(_socket) == socketIDAgentMap.end())
  {
    // Add this agent and its socket to the Map to store the initialized agents
    this->socketIDAgentMap.insert(
        std::map <int, InitMsg> ::value_type(_socket, initEffectors.front()));

    // gameState->AddAgent(playerNumber,teamName); uncomment after integration
  }


  // If the agent has been already initialized check update its effector
  if(this->socketIDAgentMap.find(_socket) != this->socketIDAgentMap.end())
  {
    // Retrieve the team name and player number for the agent belongs to the socket
    playerNumber = this->socketIDAgentMap.find(_socket)->second.playerNumber;
    teamName = this->socketIDAgentMap.find(_socket)->second.teamName;

    // update joints effector of the agent in GameState

    /* Uncomment after merging with game State
     for (const auto &team : gameState->teams)
    {
      if (team->name == teamname)
      {
        for ( auto &agent : team->members)
        {
          if (agent.uNum == playerNumber)
          {
            agent.action=this->jointEffectors;
          }
        }
      }
    }
    */

    // Update beam information for the agent

    double x, y, z;

    if (this->GetBeamInformation(x, y, z ))
    {
      // Uncomment after merging with game State
      //gameState->BeamAgent(playerNumber, teamName , x , y, z);
    }
  }

}

//////////////////////////////////////////////////
bool EffectorParser::GetSceneInformation(std::string &_msg, int &_robotType)
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
bool EffectorParser::GetInitInformation(std::string &_teamName,
    int &_playerNumber)
{
  if (!this->initEffectors.empty())
  {
    _teamName = this->initEffectors.front().teamName;
    _playerNumber = this->initEffectors.front().playerNumber;
    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool EffectorParser::GetBeamInformation(double &_x, double &_y, double &_z)
{
  if (!this->initEffectors.empty())
  {
    _x = this->beamEffectors.front().x;
    _y = this->beamEffectors.front().y;
    _z = this->beamEffectors.front().z;

    return true;
  }
  return false;
}

//////////////////////////////////////////////////
bool EffectorParser::GetJointEffector(const std::string &_jointName,
    double &_targetSpeed)
{
  std::map<std::string, double>::const_iterator it =
      this->jointEffectors.find(_jointName);

  if (it == this->jointEffectors.end()) return false;

  _targetSpeed = this->jointEffectors.find(_jointName)->second;

  return true;
}
