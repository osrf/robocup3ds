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
#include <netdb.h>
#include <string>

#include "ClientAgent.hh"

const int ClientAgent::kThreadSleepTime = 50;

//////////////////////////////////////////////////
ClientAgent::ClientAgent(const std::string &_serverAddr, const int _port,
  const int _monitorPort):
  serverAddr(_serverAddr),
  port(_port),
  monitorPort(_monitorPort),
  running(false),
  connected(false),
  reConnects(5)
{
}

//////////////////////////////////////////////////
ClientAgent::~ClientAgent()
{
  this->running = false;
  if (this->thread.joinable())
  {
    this->thread.join();
  }
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
}

//////////////////////////////////////////////////
void ClientAgent::Update()
{
  if (!this->connected && this->reConnects >= 0)
  {
    this->connected = this->Connect();
  }
  else if (!this->connected)
  {
    return;
  }

  std::cout << "client has connected!" << std::endl;
}

//////////////////////////////////////////////////
bool ClientAgent::Connect()
{
  struct sockaddr_in servaddr;
  int mySocket = socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = inet_addr(this->serverAddr.c_str());
  servaddr.sin_port = htons(this->port);

  if (connect(mySocket, (struct sockaddr *)&servaddr, sizeof(servaddr)) == 0)
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
