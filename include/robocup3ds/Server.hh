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
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <netinet/in.h>
#include <string.h>
#include <time.h>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

typedef struct {
  int sock;
  struct sockaddr address;
  socklen_t addr_len;
} connection_t;

 class Server
 {
  public: static Server *GetUniqueInstance();

  public: bool Push(const int _id, const std::string &_data);
  public: bool Pop(const int _id, std::string &_data);

  /// \brief Start and run the Server. Multithread architecture
  /// is used to handle the connection to multiple running clients.
  public: void RunReceptionTask();

   public: void Start();

  /// \brief Destructor
  public: virtual ~Server();

   /// \brief Initialize
  private: Server();

  private: static Server *uniqueInstance;

  class Client
  {
    public: Client(int _socket)
    {
      this->socket = _socket;
    }
    public: int socket;
    public: std::vector<std::string> incoming;
  };

  /// \brief Thread in charge of receiving and handling incoming messages.
  private: std::thread threadReception;
  public: std::map<int, std::shared_ptr<Client>> clients;
  private: std::atomic<bool> enabled;
  private: std::mutex mutex;

};
#endif /* INCLUDE_ROBOCUP3DS_SERVER_HH_ */
