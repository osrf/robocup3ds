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
#include "robocup3ds/ActionMessageParser.hh"

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

  // Send S_expresion.
  std::string content = "(scene rsg/agent/nao/nao_hetero.rsg 0)(he1 3.20802)(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)(lae4 0)(rae4 0)";

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

    // Create a Parser
    ActionMessageParser *parser = ActionMessageParser::GetUniqueInstance();
    parser->parseMessage(recvData, client.first);

    std::cout << "Joints Msg Parsed, he1"<< ": "
        << parser->jointParserMap[client.first][ActionMessageParser::he1]
                                           << std::endl;
    // Test joint message Parser
    EXPECT_DOUBLE_EQ(
        parser->jointParserMap[client.first][ActionMessageParser::he1], 3.20802);
    EXPECT_DOUBLE_EQ(
        parser->jointParserMap[client.first][ActionMessageParser::he2], -1.80708);
    EXPECT_DOUBLE_EQ(
        parser->jointParserMap[client.first][ActionMessageParser::lle1], 0);
    EXPECT_DOUBLE_EQ(
        parser->jointParserMap[client.first][ActionMessageParser::lae1], -0.259697);

    std::string sceneAddress;
    int rType;

    // Test Scene message Parser
    if(parser->getParsedScene(client.first,sceneAddress,rType))
    {
       std::cout << "Scene Msg Parsed: "<< sceneAddress << ", " << rType<< std::endl ;
       EXPECT_EQ(sceneAddress, "rsg/agent/nao/nao_hetero.rsg");
       EXPECT_EQ(rType, 0);
    }


  }
  close(sockfd);
}

//////////////////////////////////////////////////
TEST(ActionParser, Nima)
{
  gazebo::Server testServer(3100);

  testServer.Start();
  std::thread clientThread(&clientTask, &testServer);

  if (clientThread.joinable())
    clientThread.join();
}
