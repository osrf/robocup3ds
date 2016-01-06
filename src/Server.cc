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

#include <netinet/in.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <iostream>
#include <memory>
#include <mutex>

#include "robocup3ds/SocketParser.hh"
#include "robocup3ds/Server.hh"

//////////////////////////////////////////////////
RCPServer::RCPServer(const int _port,
                     const std::shared_ptr<SocketParser> &_parser):
  port(_port),
  masterSocket(-1),
  parser(_parser),
  enabled(false)
{}

//////////////////////////////////////////////////
RCPServer::~RCPServer()
{
  this->enabled = false;
  if (this->threadReception.joinable())
  {
    this->threadReception.join();
  }
}

//////////////////////////////////////////////////
void RCPServer::Start()
{
  // The service is already running.
  if (this->enabled)
  {
    return;
  }

  this->enabled = true;

  // Start the thread that receives information.
  this->threadReception = std::thread(&RCPServer::RunReceptionTask, this);
}

//////////////////////////////////////////////////
bool RCPServer::DisconnectClient(const int _socket)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  for (size_t i = 0; i < this->pollSockets.size(); ++i)
  {
    if (_socket == this->pollSockets.at(i).fd)
    {
      this->parser->OnDisconnection(_socket);
      close(_socket);
      this->pollSockets.at(i).events = 0;
      this->pollSockets.erase(this->pollSockets.begin() + i);
      return true;
    }
  }
  return false;
}

//////////////////////////////////////////////////
bool RCPServer::Send(const int _socket, const char *_data, const size_t _len)
{
  if (!this->enabled)
  {
    std::cerr << "RCPServer::Send() error: Service not enabled yet."
              << std::endl;
    return false;
  }

  std::lock_guard<std::mutex> lock(this->mutex);

  bool found = false;
  for (size_t i = 1; i < this->pollSockets.size(); ++i)
  {
    if (this->pollSockets.at(i).fd == _socket)
    {
      found = true;
      break;
    }
  }

  if (!found)
  {
    std::cerr << "RCPServer::Send() Socket not found." << std::endl;
    return false;
  }

  // Send data using the socket.
  auto bytes_written = write(_socket, _data, _len);
  if (bytes_written < 0)
  {
    std::cerr << "RCPServer::Send() Error writing to socket." << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
bool RCPServer::InitializeSockets()
{
  // Create the master socket.
  this->masterSocket = socket(AF_INET, SOCK_STREAM, 0);
  if (this->masterSocket == -1)
  {
    std::cerr << "Error creating master socket." << std::endl;
    return false;
  }

  // Socket option: SO_REUSEADDR.
  int value = 1;
  if (setsockopt(this->masterSocket, SOL_SOCKET, SO_REUSEADDR,
                 reinterpret_cast<const char *>(&value), sizeof(value)) != 0)
  {
    std::cerr << "Error setting socket option (SO_REUSEADDR)." << std::endl;
    close(this->masterSocket);
    return false;
  }

#ifdef SO_REUSEPORT
  // Socket option: SO_REUSEPORT.
  int reusePort = 1;
  if (setsockopt(this->masterSocket, SOL_SOCKET, SO_REUSEPORT,
                 reinterpret_cast<const char *>(&reusePort),
                 sizeof(reusePort)) != 0)
  {
    std::cerr << "Error setting socket option (SO_REUSEPORT)." << std::endl;
    return false;
  }
#endif

  // Bind the socket.
  struct sockaddr_in mySocketAddr;
  memset(&mySocketAddr, 0, sizeof(mySocketAddr));
  mySocketAddr.sin_family = AF_INET;
  mySocketAddr.sin_port = htons(this->port);
  mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(this->masterSocket, (struct sockaddr *)&mySocketAddr,
           sizeof(struct sockaddr)) < 0)
  {
    std::cerr << "Binding to a local port failed." << std::endl;
    return false;
  }

  if (listen(this->masterSocket, 5) != 0)
  {
    std::cerr << "RCPServer::InitializeSockets() Error on listen()"
              << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void RCPServer::RunReceptionTask()
{
  if (!this->InitializeSockets())
  {
    return;
  }

  // Add the master socket to the list of sockets.
  struct pollfd masterFd;
  masterFd.fd = this->masterSocket;
  masterFd.events = POLLIN;
  this->pollSockets.push_back(masterFd);

  while (this->enabled)
  {
    // Block until we receive a datagram from the network
    // (from anyone including ourselves).
    int pollReturnCode =
      poll(&this->pollSockets[0], this->pollSockets.size(), 500);

    if (pollReturnCode == -1)
    {
      std::cerr << "RCPServer::RunReceptionTask(): Polling error!" << std::endl;
      return;
    }
    else if (pollReturnCode == 0)
    {
      // Timeout occurred.
      continue;
    }

    {
      std::lock_guard<std::mutex> lock(this->mutex);

      // Data received on master socket.
      if (this->pollSockets.at(0).revents && POLLIN)
      {
        this->DispatchRequestOnMasterSocket();
        continue;
      }

      // Data received on any of the client sockets.
      this->DispatchRequestOnClientSocket();
    }
  }

  // About to leave, close pending sockets.
  for (size_t i = 1; i < this->pollSockets.size(); ++i)
  {
    close(this->pollSockets.at(i).fd);
  }
}

//////////////////////////////////////////////////
void RCPServer::DispatchRequestOnMasterSocket()
{
  // Add a new socket for this client.
  struct sockaddr_in cliAddr;
  socklen_t clilen = sizeof(cliAddr);
  int newSocketFd =
    accept(this->masterSocket, (struct sockaddr *) &cliAddr, &clilen);
  if (newSocketFd < 0)
  {
    std::cerr << "RCPServer::DispatchRequestOnMasterSocket() error on accept()"
              << std::endl;
  }

  pollfd newSocketPollItem;
  newSocketPollItem.fd = newSocketFd;
  newSocketPollItem.events = POLLIN;

  // Add the new socket to the list of sockets to poll.
  this->pollSockets.push_back(newSocketPollItem);

  // Call OnConnection().
  this->parser->OnConnection(newSocketFd);
}

//////////////////////////////////////////////////
void RCPServer::DispatchRequestOnClientSocket()
{
  for (size_t i = 1; i < this->pollSockets.size(); ++i)
  {
    if (this->pollSockets.at(i).revents && POLLIN)
    {
      int nread;
      ioctl(this->pollSockets.at(i).fd, FIONREAD, &nread);
      // The client has disconnected.
      if (nread == 0)
      {
        int socket = this->pollSockets.at(i).fd;

        // Call OnDisconnection().
        this->parser->OnDisconnection(this->pollSockets.at(i).fd);

        // Remove the client from the list used by poll.
        close(socket);
        this->pollSockets.at(i).events = 0;
        this->pollSockets.erase(this->pollSockets.begin() + i);

        break;
      }

      // The client has send some data.
      // Read data from the socket using the parser.
      if (!this->parser->Parse(this->pollSockets.at(i).fd))
      {
        std::cerr << "RCPServer::DispatchRequestOnClientSocket() error: "
                  << "Problem parsing incoming data" << std::endl;
        break;
      }

      continue;
    }
  }
}

//////////////////////////////////////////////////
int RCPServer::GetPort() const
{
  return this->port;
}

//////////////////////////////////////////////////
void RCPServer::SetPort(const int _port)
{
  this->port = _port;
}
