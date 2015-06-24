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

#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace gazebo
{
  /// \brief
  class Server
  {
    /// \brief
    class Client
    {
      /// \brief
      public: Client(int _socket)
      {
        this->socket = _socket;
      }

      /// \brief
      public: int socket;

      /// \brief
      public: std::vector<std::string> incoming;
    };

    /// \brief Constructor.
    /// \param[in] _port TCP port for incoming connections.
    public: Server(int _port);

    /// \brief Destructor
    public: virtual ~Server();

    /// \brief Push some data to be sent by the server.
    /// \param[in] _id Client ID.
    /// \param[in] _data Data to send.
    /// \return True when data was succesfully send or false otherwise.
    public: bool Push(const int _id, const std::string &_data);

    /// \brief Get some data received from the server.
    /// \param[in] _id Client ID.
    /// \param[out] _data Data received.
    /// \return True when there was data available for client ID
    /// or false otherwise.
    public: bool Pop(const int _id, std::string &_data);

    /// \brief Enable the server.
    public: void Start();

    /// \brief Task running in a different thread in charge of dispatching the
    /// new connections.
    private: void RunReceptionTask();

    /// \brief
    public: std::map<int, std::shared_ptr<Client>> clients;

    /// \brief
    private: static const int kBufferSize = 8192;

    /// \brief
    private: int port;

    /// \brief
    private: std::atomic<bool> enabled;

    /// \brief
    private: std::mutex mutex;

    /// \brief Thread in charge of receiving and handling incoming messages.
    private: std::thread threadReception;
  };
}
#endif /* INCLUDE_ROBOCUP3DS_SERVER_HH_ */
