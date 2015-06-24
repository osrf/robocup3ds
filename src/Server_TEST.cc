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

#include <netdb.h>
#include <chrono>
#include <thread>
#include "gtest/gtest.h"
#include "robocup3ds/Server.hh"

//////////////////////////////////////////////////
void clientTask(gazebo::Server *_server)
{
  // Wait some time to make sure that the server is alive.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Create a client.
  struct sockaddr_in servaddr;
  auto sockfd = socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(3100);

  connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

  // Send some data.
  std::string content = "hello";
  auto sent = write(sockfd, content.c_str(), content.size() + 1);
  EXPECT_EQ(static_cast<size_t>(sent), content.size() + 1);

  /// Wait some time until the server processes the request.
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Check that the data is available in the server.
  EXPECT_EQ(_server->clients.size(), 1u);
  for (auto client : _server->clients)
  {
    std::string recvData;
    EXPECT_TRUE(_server->Pop(client.second->socket, recvData));
    std::cout << "Data received: " << recvData << std::endl;
    EXPECT_EQ(recvData, content);
  }

  close(sockfd);
}

//////////////////////////////////////////////////
TEST(Server, Carlos)
{
  gazebo::Server testServer(3100);

  testServer.Start();
  std::thread clientThread(&clientTask, &testServer);

  if (clientThread.joinable())
    clientThread.join();
}
