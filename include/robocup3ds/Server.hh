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

 class Server {
  public: static Server *GetUniqueInstance();

  public: bool Push(const int _id, const std::string &_data);
  public: bool Pop(const int _id, std::string &_data);

  /// \brief Get the Message that send to the client.
  public: std::string GetSendingMessage();

  /// \brief Get the Message that received from the client.
  public: std::string GetRecievingMessage();

  /// \brief Get the unique number assigned to each client.
  public:int GetAgentNo();

  /// \brief Start the communication between agent and a client
  /// Using TCP protocol.
  /// \param[in] _sock TCP Socket.
  public: void DispatchRequest(int _sock);

  /// \brief Start and run the Server. Multithread architecture
  /// is used to handle the connection to multiple running clients.
  public: void Start();

  public: void Start2();

  /// \brief Destructor
  public: virtual ~Server();

  /// \brief Printing Error message and Exit the program
  /// \param[in] _msg Error Message
  public: void Error(const char *_msg)
  {
    perror(_msg);
    exit(1);
  }

  /// \brief Initialize
  private: Server();

  private: static Server *uniqueInstance;

  /// \brief Counting the clients connected to server.
  private: int clientCounter;

  /// \brief A unique number assigned to each client.
  private: int agentNo;

  /// \brief Message sent.
  private: std::stringstream sendingMessage;

  /// \brief Message received.
  private: std::string receivingMessage;

  class Client
  {
    public: Client(int _socket)
    {
      this->socket = _socket;
    }
    public: int socket;
    public: std::vector<std::string> incoming;
  };

  public: std::map<int, std::shared_ptr<Client>> clients;
  private: std::mutex mutex;

};
#endif /* INCLUDE_ROBOCUP3DS_SERVER_HH_ */
