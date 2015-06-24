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
#include <mutex>
#include "robocup3ds/Server.hh"

using namespace gazebo;

//////////////////////////////////////////////////
Server::Server()
{
  this->enabled = false;
}

Server::~Server()
{
  this->enabled = false;
  if (this->threadReception.joinable())
    this->threadReception.join();
}

void Server::Start()
{
  // The service is already running.
  if (this->enabled)
    return;

  this->enabled = true;

  // Start the thread that receives information.
  this->threadReception =
    std::thread(&Server::RunReceptionTask, this);
}

bool Server::Push(const int _id, const std::string &_data)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Sanity check: Make sure that the client ID exists.
  if (this->clients.find(_id) == this->clients.end())
  {
    std::cerr << "Server::pop() error. Client not found" << std::endl;
    return false;
  }

  // Send data using the soket.
  auto bytes_written = write(_id, _data.c_str(), _data.size() + 1);
  if (bytes_written < 0)
  {
    std::cerr << "ERROR writing to socket" << std::endl;
    return false;
  }

  return true;
}

bool Server::Pop(const int _id, std::string &_data)
{
  std::lock_guard<std::mutex> lock(this->mutex);

  // Sanity check: Make sure that the client ID exists.
  if (this->clients.find(_id) == this->clients.end())
  {
    std::cerr << "Server::pop() error. Client not found" << std::endl;
    return false;
  }

  if (this->clients[_id]->incoming.empty())
    return false;

  _data = this->clients[_id]->incoming.at(0);
  this->clients[_id]->incoming.erase(this->clients[_id]->incoming.begin());
  return true;
}

void Server::RunReceptionTask()
{
  // Create the master socket.
  int sockfd = socket(AF_INET, SOCK_STREAM, 0);

  // Socket option: SO_REUSEADDR.
  int value = 1;
  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR,
    reinterpret_cast<const char *>(&value), sizeof(value)) != 0)
  {
    std::cerr << "Error setting socket option (SO_REUSEADDR)." << std::endl;
    close(sockfd);
    return;
  }

  // Bind the socket.
  struct sockaddr_in mySocketAddr;
  memset(&mySocketAddr, 0, sizeof(mySocketAddr));
  mySocketAddr.sin_family = AF_INET;
  mySocketAddr.sin_port = htons(this->kPortNumber);
  mySocketAddr.sin_addr.s_addr = htonl(INADDR_ANY);
  if (bind(sockfd, (struct sockaddr *)&mySocketAddr,
    sizeof(struct sockaddr)) < 0)
  {
    std::cerr << "Binding to a local port failed." << std::endl;
    return;
  }

  listen(sockfd, 5);

  // Block until we receive a datagram from the network (from anyone
  // including ourselves)
  std::vector<pollfd> pollSockets;

  // Master file descriptor.
  struct pollfd masterFd;
  masterFd.fd = sockfd;
  masterFd.events = POLLIN;
  pollSockets.push_back(masterFd);

  while (this->enabled)
  {
    int pollReturnCode = poll(&pollSockets[0], pollSockets.size(), 500);
    if (pollReturnCode == -1)
    {
      std::cerr << "Server::Start2::Polling error!" << std::endl;
      return;
    }
    else if (pollReturnCode == 0)
    {
      // Timeout occurred.
      continue;
    }

    // Data received on master socket.
    if (pollSockets.at(0).revents && POLLIN)
    {
      std::cout << "New data received on master socket." << std::endl;

      // Add a new socket for this client.
      struct sockaddr_in cliAddr;
      socklen_t clilen = sizeof(cliAddr);
      int newSocketFd = accept(sockfd, (struct sockaddr *) &cliAddr, &clilen);
      if (newSocketFd < 0)
        std::cerr << "ERROR on accept" << std::endl;

      pollfd newSocketPollItem;
      newSocketPollItem.fd = newSocketFd;
      newSocketPollItem.events = POLLIN;

      {
        std::lock_guard<std::mutex> lock(this->mutex);
        pollSockets.push_back(newSocketPollItem);

        // Register the new client.
        clients[newSocketFd] = std::make_shared<Client>(newSocketFd);
      }

      continue;
    }

    // Data received on any of the client sockets.
    for (size_t i = 1; i < pollSockets.size(); ++i)
    {
      if (pollSockets.at(i).revents && POLLIN)
      {
        int nread;
        ioctl(pollSockets.at(i).fd, FIONREAD, &nread);
        if (nread == 0)
        {
          int socket = pollSockets.at(i).fd;
          // Remove the client from the list used by poll.
          std::cout << "Client " << i << " closed" << std::endl;
          close(socket);
          pollSockets.at(i).events = 0;
          pollSockets.erase(pollSockets.begin() + i);

          // Remove the client from the server list.
          {
            std::lock_guard<std::mutex> lock(this->mutex);
            this->clients.erase(socket);
          }
          break;
        }

        std::cout << "New data received on socket [" << i << "]" << std::endl;

        // Read data from the socket.
        char buffer[this->kBufferSize];
        bzero(buffer, sizeof(buffer));
        ssize_t bytes_Read =
          recv(pollSockets.at(i).fd, buffer, sizeof(buffer), 0);
        if (bytes_Read < 0)
          std::cerr << "ERROR reading from socket" << std::endl;

        std::cout << "Incoming data: " << std::string(buffer) << std::endl;

        // Register new incoming data.
        {
          std::lock_guard<std::mutex> lock(this->mutex);
          this->clients[pollSockets.at(i).fd]->incoming.push_back(
            std::string(buffer));
        }
        continue;
      }
    }
  }

  // Close pending sockets.
  for (auto client : this->clients)
    close(client.second->socket);
}
