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
#include <functional>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>
#include <vector>

#include "robocup3ds/SocketParser.hh"

/// \brief RCPServer class that will accept TCP sockets from external clients
/// (using the master socket).
/// For each external client request on the master socket, the server will
/// create a new socket to allow bidirectional communication between the
/// server and the new client.
///
///  ---------    Master socket
/// |         |<------------------ New client requests
/// |         |
/// |RCPServer|   Client1 socket
/// |         |<-----------------> Client1 data exchange
/// |         |   ClientN socket
///  --------- <-----------------> ClientN data exchange
///
/// The RCPServer API allows to send a message to a specific socket (client).
///
/// When data is available for reading on a socket, it's not always possible
/// to read it with one recv() call. Sometimes we just get a partial message
/// with a recv() call and we need a second call. Also, we might receive two
/// messages in a single recv(). Without knowing the application and the
/// format of the messages, it's hard to create a server that parses the
/// messages. This class uses the concept of a SocketParser class. This class
/// should know the format of the data sent over the wire. A SocketParser
/// class should implement a Parse() method that should be able to read from
/// the socket the correct amount of bytes. An object of type SocketParser
/// should be passed in as an argument to the RCPServer constructor.
///
/// SocketParser will also need to implement two callback methods, namely
/// OnConnection() and OnDisconnection(), which are called when a client
/// connects and disconnects from a socket respectively.
///
/// This is an example of how to instantiate a RCPServer class:
/// auto parser = std::make_shared<TrivialSocketParser>();
//  RCPServer server(kPort, parser);
class RCPServer
{
  /// \brief Constructor.
  /// \param[in] _port TCP port for incoming connections.
  /// \param[in] _parser Parser in charge of reading incoming data from
  /// the sockets.
  public: RCPServer(const int _port,
                    const std::shared_ptr<SocketParser> &_parser);

  /// \brief Simple constructor for testing purposes
  public: RCPServer() {}

  /// \brief Destructor
  public: virtual ~RCPServer();

  /// \brief Disconnects a client on a particular socket
  /// \param[in] _socket Client ID
  /// \return True if disconnection is successful
  public: bool DisconnectClient(const int _socket);

  /// \brief Push some data to be sent by the server.
  /// \param[in] _socket Client ID.
  /// \param[in] _data Data to send.
  /// \param[in] _len Data length in bytes.
  /// \return True when data was succesfully send or false otherwise.
  public: bool Send(const int _socket,
                    const char *_data,
                    const size_t _len);

  /// \brief Enable the server.
  public: void Start();

  /// \brief Get the port the server is using.
  /// \return The port number.
  public: int GetPort() const;

  /// \brief Set the port the server is using
  public: void SetPort(const int _port);

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

#endif /* _GAZEBO_ROBOCUP3DS_SERVER_HH_ */
