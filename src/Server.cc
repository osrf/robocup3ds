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

#include <iostream>
#include <mutex>
#include <poll.h>
#include <sys/ioctl.h>
#include "robocup3ds/Server.hh"

#define Port_Number 4101
#define MBUFFERSIZE 8192

//////////////////////////////////////////////////
Server* Server::GetUniqueInstance()
{
  static Server server;
  return &server;
}

//////////////////////////////////////////////////
Server::Server()
{
  this->clientCounter = 0;
  this->agentNo = 0;
  this->sendingMessage << "";
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

void Server::Start2()
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
  mySocketAddr.sin_port = htons(Port_Number);
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
  //memset(pollSockets, 0, sizeof(pollSockets));
  struct pollfd masterFd;
  masterFd.fd = sockfd;
  masterFd.events = POLLIN;
  pollSockets.push_back(masterFd);

  while (true)
  {
    //int pollReturnCode = poll(&pollSockets[0], pollSockets.size(), 50000);
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

    std::cout << "Socket activity" << std::endl;

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
        char buffer[MBUFFERSIZE];
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
}

//////////////////////////////////////////////////
void Server::Start()
{
  int sockfd, portno;
  struct sockaddr_in address;

  connection_t connection;

  sockfd = socket(AF_INET, SOCK_STREAM, 0);

  if (sockfd < 0)
    Error("ERROR opening socket");

  bzero(reinterpret_cast<char *>(&address), sizeof(address));

  portno = Port_Number;
  address.sin_family = AF_INET;
  address.sin_addr.s_addr = INADDR_ANY;
  address.sin_port = htons(portno);

  if (bind(sockfd, (struct sockaddr *) &address, sizeof(sockaddr_in)) < 0) {
    Error("ERROR on binding");
  }

  if (listen(sockfd, 5) < 0) {
    Error("ERROR on listening to the port");
  }

  while (1) {
    connection.sock = accept(sockfd, &connection.address,
        &connection.addr_len);

    if (connection.sock <= 0) {
      Error("ERROR on accept");
    }
    else
    {
      std::thread th(&Server::DispatchRequest, this, connection.sock);
      th.detach();
      clientCounter++;
    }
  }

  close(sockfd);
  return;
}

//////////////////////////////////////////////////
void Server::DispatchRequest(int _sock)
{
  this->agentNo = clientCounter;

  char buffer[MBUFFERSIZE];

  while (1) {
    bzero(buffer, sizeof(buffer));
    ssize_t bytes_Read = recv(_sock, buffer, sizeof(buffer), 0);
    if (bytes_Read < 0)
      Error("ERROR reading from socket");
    this->receivingMessage = std::string(buffer);

    // calling Parser to parse the action message
    // ActionMessageParser parserObj;
    // std::string str = string(buffer);
    // parserObj.parseMessage(str);

    this->sendingMessage
    << "(time (now 73.86))(GS (sl 0) (sr 0) (t 0.00) (pm BeforeKickOff))(hear FCPOpp 0.00 13.64 E7tVuuVsRt66.eli[KRI)(hear FCPortugal 0.00 -8.64 E7tVuuVt5tau[P6r4d6P)(GYR (n torso) (rt 1.20 -0.18 0.20))(ACC (n torso) (a 0.11 0.20 0.10))(HJ (n hj1) (ax -10.38))(HJ (n hj2) (ax 6.74))(HJ (n raj1) (ax -0.37))(HJ (n raj2) (ax 0.00))(HJ (n raj3) (ax -0.00))(HJ (n raj4) (ax 0.00))(HJ (n laj1) (ax -0.38))(HJ (n laj2) (ax -0.00))(HJ (n laj3) (ax -0.00))(HJ (n laj4) (ax 0.00))(HJ (n rlj1) (ax -0.01))(HJ (n rlj2) (ax -0.03))(HJ (n rlj3) (ax -0.00))(HJ (n rlj4) (ax -0.00))(HJ (n rlj5) (ax 0.01))(FRP (n rf) (c -0.01 -0.01 -0.01) (f -2.16 -0.93 110.61))(HJ (n rlj6) (ax -0.01))(HJ (n llj1) (ax -0.00))(HJ (n llj2) (ax 0.01))(HJ (n llj3) (ax 0.00))(HJ (n llj4) (ax 0.00))(HJ (n llj5) (ax -0.00))(FRP (n lf) (c 0.00 -0.01 -0.01) (f 1.59 -0.38 102.17))(HJ (n llj6) (ax -0.00))";

    // calling massage creator to create the action message
    // MessageCreator msgObj(agentNo);
    // sendingMessage=msgObj.getMessage();

    ssize_t bytes_written = write(_sock, this->sendingMessage.str().c_str(),
        MBUFFERSIZE);
    if (bytes_written < 0)
      Error("ERROR writing to socket");
  }
  close(_sock);
}

//////////////////////////////////////////////////
std::string Server::GetSendingMessage()
{
  return (this->sendingMessage.str());
}

//////////////////////////////////////////////////
std::string Server::GetRecievingMessage()
{
  return this->receivingMessage;
}

//////////////////////////////////////////////////
int Server::GetAgentNo()
{
  return this->agentNo;
}

//////////////////////////////////////////////////
Server::~Server()
{
}
