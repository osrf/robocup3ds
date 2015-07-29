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

#include <arpa/inet.h>
#include <unistd.h>
#include <string>

#include "ClientAgent.hh"

const int ClientAgent::kThreadSleepTime = 20;

//////////////////////////////////////////////////
ClientAgent::ClientAgent(const std::string &_serverAddr, const int _port,
                         const int _monitorPort):
  running(false),
  connected(false),
  serverAddr(_serverAddr),
  port(_port),
  monitorPort(_monitorPort),
  socketID(-1),
  monitorSocketID(-1),
  reConnects(5)
{
}

//////////////////////////////////////////////////
ClientAgent::~ClientAgent()
{
  this->running = false;
  while (!this->thread.joinable())
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(kThreadSleepTime));
  }
  this->thread.join();
  std::cout << "client shutting down!" << std::endl;
}

//////////////////////////////////////////////////
void ClientAgent::Start()
{
  // The service is already running.
  if (this->running)
  { return; }

  this->running = true;

  // Start the thread that receives information.
  this->thread = std::thread(&ClientAgent::Update, this);
  std::cout << "client running!" << std::endl;
}

//////////////////////////////////////////////////
void ClientAgent::Update()
{
  while (this->reConnects >= 0 && !this->connected)
  {
    this->connected = this->Connect(this->port, this->socketID);
  }
  if (!this->connected)
  { return; }

  std::cout << "client has connected!" << std::endl;

  while (this->running)
  {
    std::this_thread::sleep_for(
      std::chrono::milliseconds(kThreadSleepTime));
    this->InitAndBeam();
  }
}

//////////////////////////////////////////////////
bool ClientAgent::Connect(const int &_port, int &_socketID)
{
  struct sockaddr_in servaddr;
  _socketID = socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(this->serverAddr.c_str());
  servaddr.sin_port = htons(_port);

  if (connect(_socketID, (struct sockaddr *)&servaddr, sizeof(servaddr)) == 0)
  {
    return true;
  }
  else
  {
    this->reConnects--;
    std::cerr << "cannot connect to server, "
              << this->reConnects << " tries left!" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(kThreadSleepTime));
    return false;
  }
}

void ClientAgent::InitAndBeam()
{
  static bool init = false;
  std::string msg;

  if (!init)
  {
    msg = "(init (unum 0) (teamname red))(beam 1 1 90)";
    if (this->PutMessage(msg))
    {
      init = true;
      std::cout << "client sent init msg!" << std::endl;
    }
  }
}


bool ClientAgent::PutMessage(const std::string &_msg)
{
  if (_msg.empty() || !this->connected)
  {
    return false;
  }

  // prefix the message with it's payload length
  unsigned int len = htonl(_msg.size());
  std::string prefix((const char *)&len, sizeof(unsigned int));
  std::string str = prefix + _msg;
  if (static_cast<ssize_t>(str.size()) != write(this->socketID, str.data(),
      str.size()))
  {
    std::cerr << "could not put entire message: " + _msg << std::endl;
    return false;
  }

  return true;
}

bool ClientAgent::PutMonMessage(const std::string &_msg)
{
  if (_msg.empty() || !this->connected)
  {
    return false;
  }

  // prefix the message with it's payload length
  unsigned int len = htonl(_msg.size());
  std::string prefix((const char *)&len, sizeof(unsigned int));
  std::string str = prefix + _msg;
  if (static_cast<ssize_t>(str.size()) != write(this->monitorSocketID,
      str.data(), str.size()))
  {
    std::cerr << "could not put entire monitor message: " + _msg << std::endl;
    return false;
  }

  return true;
}

bool ClientAgent::GetMessage(std::string &_msg)
{
  static char buffer[16 * 1024];

  unsigned int bytesRead = 0;
  while (bytesRead < sizeof(unsigned int))
  {
    // SelectInput();
    int readResult = read(this->socketID, buffer + bytesRead,
                          sizeof(unsigned int) - bytesRead);
    if (readResult < 0)
    { continue; }
    // if (readResult == 0)
    // {
    //   // [patmac] Kill ourselves if we disconnect from the server
    //   // for instance when the server is killed.  This helps to
    //   // prevent runaway agents.
    //   cerr << "Lost connection to server" << endl;
    //   Done();
    //   exit(1);
    // }
    bytesRead += readResult;
  }

  // msg is prefixed with it's total length
  union int_char_t
  {
    char *c;
    unsigned int *i;
  };
  int_char_t size;
  size.c = buffer;
  unsigned int msgLen = ntohl(*(size.i));
  // cerr << "GM 6 / " << msgLen << " (bytesRead " << bytesRead << ")\n";
  if (sizeof(unsigned int) + msgLen > sizeof(buffer))
  {
    std::cerr << "too long message; aborting" << std::endl;
    abort();
  }

  // read remaining message segments
  unsigned int msgRead = bytesRead - sizeof(unsigned int);

  char *offset = buffer + bytesRead;

  while (msgRead < msgLen)
  {
    // if (!SelectInput())
    // {
    //   return false;
    // }

    unsigned readLen = sizeof(buffer) - msgRead;
    if (readLen > msgLen - msgRead)
    { readLen = msgLen - msgRead; }

    int readResult = read(this->socketID, offset, readLen);
    if (readResult < 0)
    { continue; }
    msgRead += readResult;
    offset += readResult;
  }

  // zero terminate received data
  (*offset) = 0;

  _msg = std::string(buffer + sizeof(unsigned int));

  // static std::string lastMsg = "";
  // if (msg.compare(lastMsg) == 0)
  // {
  //   cerr << "Duplicate message received from server"
  //        "-- has the server killed us?\n";
  //   Done();
  //   exit(1);
  // }
  // lastMsg = msg;

  return true;
}
