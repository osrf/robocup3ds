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

#ifndef _GAZEBO_ROBOCUP3DS_SOCKETPARSER_HH_
#define _GAZEBO_ROBOCUP3DS_SOCKETPARSER_HH_

/// \brief Interface class designed to interact with the Server class. The
/// Server constructor accepts a SocketParser instance as an argument in its
/// constructor. Users of the Server class will have to create its own parsers
/// to be able to read data from the sockets. Users are the only ones that know
/// how to read and interpret the data sent through the sockets. The server will
/// call the Parse() method of the derived parser class that the users will
/// write. The server will call the OnConnection() method when it receives a
/// new connection through a socket, and similarly call the OnDisconnection()
/// method when the connection is lost.
class SocketParser
{
  /// \brief Constructor.
  public: SocketParser() = default;

  /// \brief Destructor
  public: virtual ~SocketParser() = default;

  /// \brief Parse some data from a socket.
  /// \param[in] _socket Socket to read.
  /// \return True when data was succesfully parsed or false otherwise.
  public: virtual bool Parse(const int _socket) = 0;

  /// \brief Callback function when new connection is created
  /// \param[in] _socket Socket to read.
  public: virtual void OnConnection(const int _socket) = 0;

  /// \brief Callback function when existing connection is disconnected
  /// \param[in] _socket Socket to read.
  public: virtual void OnDisconnection(const int _socket) = 0;
};

#endif /* _GAZEBO_ROBOCUP3DS_SOCKETPARSER_HH_ */
