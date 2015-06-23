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

#ifndef _GAZEBO_ROBOCUP3DS_SERVER_HH_
#define _GAZEBO_ROBOCUP3DS_SERVER_HH_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <string.h>
#include <time.h>
#include <string>
#include <sstream>
#include <thread>

typedef struct {
  int sock;
  struct sockaddr address;
  socklen_t addr_len;
} connection_t;

class Server {
  private: static Server *uniqueInstance;

  private: int clientCounter, agentNo;

  private: std::stringstream sendingMessage;

  private: std::string receivingMessage;

  private: Server();

  public: static Server *GetUniqueInstance();

  public: std::string GetSendingMessage();

  public: std::string GetRecievingMessage();

  public:int GetAgentNo();

  public: void MessagePassing(int _sock);

  public: void Start();

  public: virtual ~Server();

  public: void Error(const char *msg)
  {
    perror(msg);
    exit(1);
  }
};
#endif /* INCLUDE_ROBOCUP3DS_SERVER_HH_ */
