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
#include <errno.h>
#include <fcntl.h>
#include <unistd.h>
#include <ignition/math.hh>
#include <mutex>
#include <string>
#include <utility>
#include "ClientAgent.hh"

using namespace ignition;

const int ClientAgent::kThreadSleepTime = 20000;

//////////////////////////////////////////////////
ClientAgent::ClientAgent(const std::string &_serverAddr, const int _port,
                         const int _monitorPort,
                         const int _uNum, const std::string &_teamName,
                         const std::string &_side):
  running(false),
  connected(false),
  cycleCounter(0),
  uNum(_uNum),
  teamName(_teamName),
  side(_side),
  serverAddr(_serverAddr),
  port(_port),
  monitorPort(_monitorPort),
  socketID(-1),
  monitorSocketID(-1),
  reConnects(6),
  verbose(false)
{
}

//////////////////////////////////////////////////
ClientAgent::~ClientAgent()
{
  if (this->running)
  {
    while (!this->thread.joinable())
    {
      this->Wait();
    }
    this->running = false;
    this->thread.join();
  }
  std::cout << "client shutting down!" << std::endl;
}

//////////////////////////////////////////////////
void ClientAgent::Wait(const int _msec)
{
  std::this_thread::sleep_for(std::chrono::microseconds(_msec));
}

//////////////////////////////////////////////////
void ClientAgent::Start()
{
  // The service is already running.
  if (this->running)
  {
    return;
  }

  this->running = true;

  // Start the thread that receives information.
  this->thread = std::thread(&ClientAgent::Update, this);
  std::cout << "client running!" << std::endl;
}

//////////////////////////////////////////////////
void ClientAgent::Update()
{
  bool clientConnect = false;
  bool monitorConnect = false;
  while (this->reConnects > 0 && !this->connected)
  {
    this->Wait();
    if (!clientConnect)
    {
      clientConnect = this->Connect(this->port, this->socketID);
    }
    if (!monitorConnect)
    {
      monitorConnect =
        this->Connect(this->monitorPort, this->monitorSocketID);
    }
    this->connected = clientConnect && monitorConnect;
  }
  if (!this->connected)
  {
    return;
  }

  std::cout << "client has connected!" << std::endl;

  size_t currActionIndex = 0;
  size_t currMsgIndex = 0;
  // std::chrono::microseconds ms(20000);
  std::string receivedMsg;
  while (this->running)
  {
    this->Wait();
    // std::chrono::high_resolution_clock::time_point start =
    //   std::chrono::high_resolution_clock::now();

    if (this->verbose)
    {
      std::cerr << std::endl;
      std::cerr << "current cycle: " << this->cycleCounter << std::endl;
    }

    receivedMsg.clear();
    std::lock_guard<std::mutex> lock(this->mutex);
    if (this->cycleCounter > 0)
    {
      if (this->GetMessage(receivedMsg))
      {
        this->allMsgs.push_back(receivedMsg);
        if (this->verbose)
        {
          std::cerr << "received msg: " << receivedMsg << std::endl;
        }
      }
      else
      {
        std::cerr << "error receiving msg!" << std::endl;
      }
    }

    if (currActionIndex == this->actionResponses.size())
    {
      this->cycleCounter++;
      continue;
    }

    auto &ar = this->actionResponses[currActionIndex];
    ar.status = ActionResponse::Status::CURRENT;

    bool succ1 = this->PutMessage(ar.msgToSend[currMsgIndex] + " (syn)");
    bool succ2 = this->PutMonMessage(ar.monitorMsgToSend[currMsgIndex] +
                                     " (syn)");
    // std::cerr << currMsgIndex << " " << succ1 << " " << succ2 << std::endl;
    if (succ1 && succ2)
    {
      std::cerr << "[" << this->cycleCounter <<
                "] client has sent the following action: " +
                ar.actionName << std::endl;
      if (verbose)
      {
        std::cerr << "sent client msg: " << ar.msgToSend[currMsgIndex]
                  << std::endl;
        std::cerr << "sent monitor msg: " << ar.monitorMsgToSend[currMsgIndex]
                  << std::endl;
      }
      currMsgIndex++;
    }
    else
    {
      std::cerr << "error sending msg, retrying!" << std::endl;
    }

    if (receivedMsg.length() > 0)
    {
      ar.msgReceived.push_back(receivedMsg);
    }

    if (currMsgIndex == ar.msgToSend.size())
    {
      currActionIndex++;
      currMsgIndex = 0;
      ar.status = ActionResponse::Status::FINISHED;
      // std::cerr << currActionIndex << " " <<
      //           this->actionResponses.size() << std::endl;
    }
    this->cycleCounter++;
    // ms = std::chrono::duration_cast<std::chrono::microseconds>
    //      (std::chrono::high_resolution_clock::now() - start);
    // std::cerr << ms.count() << std::endl;
  }
}

//////////////////////////////////////////////////
bool ClientAgent::Connect(const int &_port, int &_socketID)
{
  struct sockaddr_in servaddr;
  _socketID = socket(AF_INET, SOCK_STREAM, 0);
  // fcntl(_socketID, F_SETFL, O_NONBLOCK);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(this->serverAddr.c_str());
  servaddr.sin_port = htons(_port);

  int r = connect(_socketID, (struct sockaddr *)&servaddr, sizeof(servaddr));
  if (r == 0)
  {
    return true;
  }
  else
  {
    this->reConnects--;
    std::cerr << "error: " << strerror(errno) <<
              ", cannot connect to server on port " << _port << ", "
              << this->reConnects << " tries left!" << std::endl;
    return false;
  }
}

//////////////////////////////////////////////////
void ClientAgent::InitAndBeam(const double _x,
                              const double _y, const double _yaw)
{
  const auto msg = "(init (unum " + std::to_string(this->uNum)
                   + ") (teamname " + this->teamName + ")) (beam "
                   + std::to_string(_x)
                   + " " + std::to_string(_y) + " "
                   + std::to_string(_yaw) + ")";

  ActionResponse ar("InitAndBeam_" + std::to_string(this->cycleCounter));
  ar.msgToSend.push_back(msg);
  ar.monitorMsgToSend.push_back("");

  std::lock_guard<std::mutex> lock(this->mutex);
  this->actionResponses.push_back(ar);
}

/////////////////////////////////////////////////
void ClientAgent::Dribble(const math::Vector3<double> &_start,
                          const math::Vector3<double> &_end, const int _nSteps)
{
  for (int i = 0; i <= _nSteps; ++i)
  {
    const auto pt = _start + ((_end - _start) * (i / _nSteps));
    auto msg = "(agent (unum " + std::to_string(this->uNum) +  ") (team "
                     + this->side + ") (pos " + std::to_string(pt.X()) +
                     " " + std::to_string(pt.Y()) +
                     " " + std::to_string(pt.Z()) + "))";
    msg += " (ball (pos " + std::to_string(pt.X()) +
           " " + std::to_string(pt.Y()) +
           " " + std::to_string(pt.Z()) + "))";
    ActionResponse ar("Dribble_" + std::to_string(this->cycleCounter));
    ar.msgToSend.push_back("");
    ar.monitorMsgToSend.push_back(msg);

    std::lock_guard<std::mutex> lock(this->mutex);
    this->actionResponses.push_back(ar);
  }
}

/////////////////////////////////////////////////
void ClientAgent::ChangePlayMode(const std::string &_playMode)
{
  const auto msg = "(playMode " + _playMode + ")";

  ActionResponse ar("PlayMode_" + std::to_string(this->cycleCounter));
  ar.msgToSend.push_back("");
  ar.monitorMsgToSend.push_back(msg);

  std::lock_guard<std::mutex> lock(this->mutex);
  this->actionResponses.push_back(ar);
}

/////////////////////////////////////////////////
void ClientAgent::MoveBall(const math::Vector3<double> &_pos)
{
  const auto msg = "(ball (pos " + std::to_string(_pos.X()) +
                   " " + std::to_string(_pos.Y()) +
                   " " + std::to_string(_pos.Z()) + "))";

  ActionResponse ar("MoveBall_" + std::to_string(this->cycleCounter));
  ar.msgToSend.push_back("");
  ar.monitorMsgToSend.push_back(msg);

  std::lock_guard<std::mutex> lock(this->mutex);
  this->actionResponses.push_back(ar);
}

/////////////////////////////////////////////////
void ClientAgent::MoveAgent(const math::Vector3<double> &_pos)
{
  const auto msg = "(agent (unum " + std::to_string(this->uNum) +  ") (team "
                   + this->side + ") (pos " + std::to_string(_pos.X()) +
                   " " + std::to_string(_pos.Y()) +
                   " " + std::to_string(_pos.Z()) + "))";

  ActionResponse ar("MoveAgent_" + std::to_string(this->cycleCounter));
  ar.msgToSend.push_back("");
  ar.monitorMsgToSend.push_back(msg);

  std::lock_guard<std::mutex> lock(this->mutex);
  this->actionResponses.push_back(ar);
}

/////////////////////////////////////////////////
void ClientAgent::RemoveAgent()
{
  const auto msg = "(kill (unum " + std::to_string(this->uNum) +  ") (team "
                   + this->side + "))";

  ActionResponse ar("KillAgent_" + std::to_string(this->cycleCounter));
  ar.msgToSend.push_back("");
  ar.monitorMsgToSend.push_back(msg);

  std::lock_guard<std::mutex> lock(this->mutex);
  this->actionResponses.push_back(ar);
}

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
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

//////////////////////////////////////////////////
bool ClientAgent::SelectInput()
{
  fd_set readfds;
  struct timeval tv;
  tv.tv_sec = 0;
  tv.tv_usec = 250000;
  FD_ZERO(&readfds);
  FD_SET(this->socketID, &readfds);

  while (true)
  {
    switch (select(this->socketID + 1, &readfds, 0, 0, &tv))
    {
    case 1:
      return true;
    case 0:
      std::cerr << "(SelectInput) select failed "
                << strerror(errno) << std::endl;
      return false;
    default:
      if (errno == EINTR)
      {
        continue;
      }
      std::cerr << "(SelectInput) select failed "
                << strerror(errno) << std::endl;
      return false;
    }
  }
}

//////////////////////////////////////////////////
bool ClientAgent::GetMessage(std::string &_msg)
{
  static char buffer[16 * 1024];

  unsigned int bytesRead = 0;
  while (bytesRead < sizeof(unsigned int))
  {
    if (!this->SelectInput())
    {
      return false;
    }
    int readResult = read(this->socketID, buffer + bytesRead,
                          sizeof(unsigned int) - bytesRead);
    if (readResult < 0)
    {
      continue;
    }
    if (readResult == 0)
    {
      return false;
    }
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
    return false;
  }

  // read remaining message segments
  unsigned int msgRead = bytesRead - sizeof(unsigned int);

  char *offset = buffer + bytesRead;

  while (msgRead < msgLen)
  {
    if (!this->SelectInput())
    {
      return false;
    }

    unsigned readLen = sizeof(buffer) - msgRead;
    if (readLen > msgLen - msgRead)
    {
      readLen = msgLen - msgRead;
    }

    int readResult = read(this->socketID, offset, readLen);
    if (readResult < 0)
    {
      continue;
    }
    msgRead += readResult;
    offset += readResult;
  }

  // zero terminate received data
  (*offset) = 0;

  _msg = std::string(buffer + sizeof(unsigned int));
  bzero(buffer, sizeof(buffer));
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
