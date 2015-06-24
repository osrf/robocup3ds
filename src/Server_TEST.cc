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
#include "gtest/gtest.h"
#include "Server.cc"

//////////////////////////////////////////////////
//std::string msgFromServer;
//void TestEQ();
//void ServerProcess();
//void AgentProcess();

//////////////////////////////////////////////////
/*TEST(Server, Simple)
{
  std::thread serverThread(&ServerProcess);
  serverThread.detach();
  sleep(1);
  std::thread agentThread(&AgentProcess);
  agentThread.detach();
//  sleep(2);
  std::thread testThread(&TestEQ);
  testThread.detach();
}*/

void clientTask(Server *_server)
{
  // Wait some time to make sure that the server is alive.
  std::this_thread::sleep_for(std::chrono::milliseconds(300));

  // Create a client.
  struct sockaddr_in servaddr;
  auto sockfd = socket(AF_INET,SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(4101);

  connect(sockfd, (struct sockaddr *)&servaddr, sizeof(servaddr));

  // Send some data.
  std::string content = "hello";
  auto sent = write(sockfd, content.c_str(), content.size() + 1);
  EXPECT_EQ(static_cast<size_t>(sent), content.size() + 1);

  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Check that the data arrived.
  EXPECT_EQ(_server->clients.size(), 1u);
  for (auto client : _server->clients)
  {
    std::string recvData;
    EXPECT_TRUE(_server->Pop(client.second->socket, recvData));
    std::cout << "Data received: " << recvData << std::endl;
    EXPECT_EQ(recvData, content);
  }

  std::cout << "Client task leaving" << std::endl;
}

//////////////////////////////////////////////////
TEST(Server, Carlos)
{
  Server *testServer = Server::GetUniqueInstance();

  std::thread clientThread(&clientTask, testServer);
  testServer->Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  std::cout << "Deleting server" << std::endl;
  delete testServer;
}

//////////////////////////////////////////////////
/*void TestEQ()
{
  // test case for singleton class
  Server* server= Server::GetUniqueInstance();
  // test the message send to server
  EXPECT_EQ("(he1 3.20802)(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)(lae4 0)(rae4 0)",
      server->GetRecievingMessage());
  // test the message send to agent
  EXPECT_EQ(server->GetSendingMessage(), msgFromServer);
}

//////////////////////////////////////////////////
void ServerProcess()
{
  Server* server= Server::GetUniqueInstance();
  server->Start();
}

//////////////////////////////////////////////////
void AgentProcess()
{
  int sockfd, portno;
  struct sockaddr_in serv_addr;
  struct hostent *serverAdd;

  char buffer[MBUFFERSIZE];

  portno = Port_Number;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    perror("ERROR opening socket");
    exit(0);
  }
  serverAdd = gethostbyname("localhost");

  if (serverAdd == NULL) {
    perror("ERROR, no such host");
    exit(0);
  }

  bzero(reinterpret_cast<char*> ( &serv_addr), sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  bcopy(reinterpret_cast<char*> (serverAdd->h_addr),
      reinterpret_cast<char*>(&serv_addr.sin_addr.s_addr),
      serverAdd->h_length);

  serv_addr.sin_port = htons(portno);

  if (connect(sockfd, (struct sockaddr *) &serv_addr, sizeof(serv_addr))< 0) {
    perror("ERROR connecting");
    exit(0);
  }

  while (1) {
    int msgLength;
    bzero(buffer, sizeof(buffer));

    snprintf(buffer, sizeof(buffer),
      "(he1 3.20802)(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)(lae4 0)(rae4 0)");

    msgLength = send(sockfd, buffer, strlen(buffer), 0);

    if (msgLength < 0) {
      perror("ERROR writing to socket");
      exit(0);
    }
    bzero(buffer, sizeof(buffer));
    usleep(2000000);
    msgLength = read(sockfd, buffer, sizeof(buffer));
    if (msgLength < 0) {
      perror("ERROR reading from socket");
      exit(0);
    }
    msgFromServer = std::string(buffer);
  }
  close(sockfd);
  return;
}*/
