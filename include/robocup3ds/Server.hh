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

#include <poll.h>
#include <atomic>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>
#include "robocup3ds/SocketParser.hh"

namespace gazebo
{
  /// \brief Server class that will accept TCP sockets from external clients
  /// (using the master socket).
  /// For each external client request on the master socket, the server will
  /// create a new socket to allow bidirectional communication between the
  /// server and the new client.
  ///
  ///  ------    Master socket
  /// |      |<------------------ New client requests
  /// |      |
  /// |Server|   Client1 socket
  /// |      |<-----------------> Client1 data exchange
  /// |      |   ClientN socket
  ///  ------ <-----------------> ClientN data exchange
  ///
  /// For each client connected, the Server class will keep a data structure
  /// named Client. This data structure will queue the incoming messages coming
  /// from the clients. The public API of the Server class will allow to pop
  /// the first message received for a given client. Symmetrically, the Server
  /// API allows to send (push) a message to a specific client.
  ///
  /// When data is available for reading on a socket, it's not always possible
  /// to read it with one recv() call. Sometimes we just get a partial message
  /// with a recv() call and we need a second call. Also, we might receive two
  /// messages in a single recv(). Without knowing the application and the
  /// format of the messages, it's hard to create a server that parses the
  /// messages. This class uses the concept of a SocketParser class. This class
  /// should know the format of the data send over the wire. A SocketParser
  /// class should implement a Parse() method that should be able to return an
  /// entire message. An object of type SocketParser should be passed in as an
  /// argument to the Server constructor.
  class Server
  {
    /// \brief Class that stores the socket connection to a given client.
    /// It also uses a queue to store the incoming messages from the client.
    class Client
    {
      /// \brief Class constructor.
      /// \param[in] _socket Existing socket with a client
      public: Client(const int _socket)
        : socket(_socket)
      {
      }

      /// \brief Class destructor.
      public: ~Client() = default;

      /// \brief TCP client socket.
      public: int socket;

      /// \brief Queue used to store message coming from the clients.
      public: std::queue<std::string> incoming;
    };

    /// \brief Constructor.
    /// \param[in] _port TCP port for incoming connections.
    /// \param[in] _parser Parser in charge of reading incoming data from
    /// the sockets.
    public: Server(const int _port,
                   const std::shared_ptr<SocketParser> &_parser);

    /// \brief Destructor
    public: virtual ~Server();

    /// \brief Push some data to be sent by the server.
    /// \param[in] _id Client ID.
    /// \param[in] _data Data to send.
    /// \return True when data was succesfully send or false otherwise.
    public: bool Push(const int _id, const std::string &_data);

    /// \brief Get some data received from the server. No that the data removed
    /// will be the oldest message received.
    /// \param[in] _id Client ID.
    /// \param[out] _data Data received.
    /// \return True when there was data available for client ID
    /// or false otherwise.
    public: bool Pop(const int _id, std::string &_data);

    /// \brief Get the list of client IDs connected at this moment.
    /// \return A Vector of client IDs.
    public: std::vector<int> GetClientIds() const;

    /// \brief Enable the server.
    public: void Start();

    /// \brief Task running in a different thread in charge of dispatching the
    /// new connections.
    private: void RunReceptionTask();

    /// \brief Dispatch a new request received on the master socket. This will
    /// require to create a new socket and add it to the list of sockets to poll
    private: void DispatchRequestOnMasterSocket();

    /// brief Dispatch a new request received on a client socket. This will
    /// require to read the data and store it and the queue for this specific
    /// client.
    private: void DispatchRequestOnClientSocket();

    /// \brief Initialize sockets, options and start accepting connections.
    /// \return true when success or false otherwise.
    private: bool InitializeSockets();

    /// \brief Map to keep track of all the clients currently connected.
    /// The key is the socket ID and the value is a pointer to the client.
    public: std::map<int, std::shared_ptr<Client>> clients;

    /// \brief Maximum size of each message received.
    private: static const int kBufferSize = 8192;

    /// \brief TCP master port used for the initial client requests.
    private: int port;

    /// \brief TCP socket where new clients can make requests.
    private: int masterSocket;

    /// \brief Vector of pollfd entries with the list of sockets to poll
    /// for activity.
    private: std::vector<pollfd> pollSockets;

    /// \brief Parser used to read incoming messages.
    private: std::shared_ptr<SocketParser> parser;

    /// \brief Flag used to detect when we have to destroy the server.
    private: std::atomic<bool> enabled;

    /// \brief Protect concurrent access.
    private: mutable std::mutex mutex;

    /// \brief Thread in charge of receiving and handling incoming messages.
    private: std::thread threadReception;
  };
}
#endif /* _GAZEBO_ROBOCUP3DS_SERVER_HH_ */
