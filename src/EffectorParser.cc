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
#include "robocup3ds/EffectorParser.hh"

EffectorParser::EffectorParser()
{
}

//////////////////////////////////////////////////
bool EffectorParser::Parse(int _socket)
{

  char buffer[this->kBufferSize];

  bzero(buffer, sizeof(buffer));

  int bytesRead = 0;

  int totalBytes;

  // Used to read the size of the message in little-endian format
  int endiannessSize=recv(_socket, buffer, 4, 0);

  if(endiannessSize<1)
  {
    return false;
  }

  // Calculate size of the s-expression messages
  totalBytes = ntohl(*(unsigned int*) buffer);

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

  // Store the received s-expression messages
  this->message << std::string(buffer);

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
      v = _exp->list->val;
    else
      return;
  }
  else
    return;

  // Decide based on the type of effector message
  if (!strcmp(v, "scene"))
  {
    ParseScene(_exp);
  }
  else if (!strcmp(v, "beam"))
  {
    ParseBeam(_exp);
  }
  else if (!strcmp(v, "init"))
  {
    ParseInit(_exp);
  }
  else if (!strcmp(v, "he1"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "he2"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle1"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle1"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle2"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle2"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle3"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle3"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle4"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle4"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle5"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle5"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lle6"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rle6"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae1"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae1"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae2"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae2"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae3"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae3"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "lae4"))
  {
    ParseHingeJoint(_exp);
  }
  else if (!strcmp(v, "rae4"))
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

  this->jointEffectors.insert(std::map <std::string, double> ::value_type(name, effector));
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
  double x,y,z = 0;

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

  this->initEffectors.push_back(InitMsg(playerNum, teamName));
}

//////////////////////////////////////////////////
void EffectorParser::OnConnection(const int _socket)
{
  this->socketID = _socket;
  this->newConnectionDetected = true;
}

//////////////////////////////////////////////////
void EffectorParser::OnDisconnection(const int /*_socket*/)
{
  this->newDisconnectionDetected = true;
}
//////////////////////////////////////////////////
void EffectorParser::Update(){
  // clear data structures
  this->beamEffectors.clear();
  this->initEffectors.clear();
  this->sceneEffectors.clear();
  this->jointEffectors.clear();

  //Update Effectors using message received by Parse()
  ParseMessage(this->message.str());

}
//////////////////////////////////////////////////
bool EffectorParser::GetSceneInformation(std::string &_msg, int &_robotType)
{
  if (!this->sceneEffectors.empty())
  {
    _msg=this->sceneEffectors.front().rsgAddress;
    _robotType=this->sceneEffectors.front().robotType;
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
    _teamName=this->initEffectors.front().teamName;
    _playerNumber=this->initEffectors.front().playerNumber;
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
EffectorParser::~EffectorParser()
{
}
