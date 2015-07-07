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
#include <condition_variable>
#include <mutex>
#include <thread>
#include "gtest/gtest.h"
#include "robocup3ds/Server.hh"
#include "robocup3ds/SocketParser.hh"
#include "robocup3ds/ActionMessageParser.hh"


const std::string content = "(scene rsg/agent/nao/nao_hetero.rsg 0)(beam 1 1 0.4)"
           "(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)"
           "(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)"
           "(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)";

const int kPort = 6234;
std::mutex mutex;
std::condition_variable cv;
int socket_ID;
bool clientReady = false;
bool serverReady = false;

//////////////////////////////////////////////////
void reset()
{
  clientReady = false;
  serverReady = false;
  ActionMessageParser actionObj;
  actionObj.newConnectionDetected=false;
  actionObj.newDisconnectionDetected=false;
}

//////////////////////////////////////////////////
void SendSomeData(int _client_socket)
{
  char mBuffer[8090];
  char* out = (char*) content.c_str();
  strcpy(mBuffer + 4, out);
  unsigned int len = strlen(out) + 1;
  unsigned int netlen = htonl(len);
  memcpy(mBuffer, &netlen, 4);

  std::cout << "msgSent \"" << content << "\"\n\n";
  std::cout << "msglen:"<<len <<std::endl;


  size_t sent = write(_client_socket, mBuffer, content.size() + 4);
  fsync(_client_socket);
  EXPECT_EQ(sent, content.size() + 4u);
}

//////////////////////////////////////////////////
bool createClient(const int _port, int &_socket)
{
  struct sockaddr_in servaddr;
  _socket = socket(AF_INET, SOCK_STREAM, 0);

  bzero(&servaddr, sizeof(servaddr));
  servaddr.sin_family = AF_INET;
  servaddr.sin_addr.s_addr = htonl(INADDR_ANY);
  servaddr.sin_port = htons(_port);

  if (connect(_socket, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
  {
    std::cerr << "createClient::connect() error" << std::endl;
    return false;
  }

  return true;
}

//////////////////////////////////////////////////
void senderClient(const int _port)
{
  int socket;
  if (!createClient(_port, socket))
    return;

  // Send some data.
  auto sent = write(socket, content.c_str(), content.size() + 1);
  EXPECT_EQ(static_cast<size_t>(sent), content.size() + 1);

  clientReady = true;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cv.notify_one();

  // Wait some time until the server checks results.
  {
    std::unique_lock<std::mutex> lk(mutex);
    auto now = std::chrono::system_clock::now();
    if (!cv.wait_until(lk, now + std::chrono::milliseconds(500),
          [](){return serverReady;}))
    {
      FAIL();
    }
  }

  close(socket);
}

//////////////////////////////////////////////////
void receiverClient(const int _port)
{
  int socket;
  
  if (!createClient(_port, socket))
    return;

  // Send some data to trigger the creation of a new client.
  auto sent = write(socket, content.c_str(), content.size() + 1);
  EXPECT_EQ(static_cast<size_t>(sent), content.size() + 1);

  clientReady = true;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cv.notify_one();


  // Receive some data.
  auto parser = std::make_shared<ActionMessageParser>();
  EXPECT_TRUE(parser->Parse(socket));
  EXPECT_EQ(parser->message.str(),content);
  std::cout<<"message recieved:"<<parser->message.str()<<std::endl;
  close(socket);
}

//////////////////////////////////////////////////
TEST(Server, Parser)
{
  reset();

  auto parser = std::make_shared <ActionMessageParser>();

  gazebo::Server server(kPort, parser,
    &ActionMessageParser::OnConnection, parser.get(),
    &ActionMessageParser::OnDisconnection, parser.get());

  server.Start();

  std::thread clientThread(&receiverClient, kPort);

  // Wait some time until the client sends the request.
  {
    std::unique_lock<std::mutex> lk(mutex);
    auto now = std::chrono::system_clock::now();
    if (!cv.wait_until(lk, now + std::chrono::milliseconds(500),
          [](){return clientReady;}))
    {
      FAIL();
      return;
    }
  }

  // Check that the callback for detecting new connections was executed.
  EXPECT_TRUE(parser->newConnectionDetected);

  // Send some data to the client.
  SendSomeData(parser->socket);

  // Test Scene message Parser
  std::string sceneAddress;
  int rType;

  if ( parser->getSceneInformation ( parser->socket, sceneAddress, rType ) )
  {
    std::cout << "Scene Msg Parsed: " << sceneAddress
        << ", " << rType << std::endl;
    EXPECT_EQ(sceneAddress, "rsg/agent/nao/nao_hetero.rsg");
    EXPECT_EQ(rType, 0);
  }

  // Test Init message Parser
  std::string teamname;
  int playerNumber;

  if ( parser->getInitInformation(parser->socket, teamname, playerNumber) )
  {
    std::cout << "Init Msg Parsed: "<< teamname << ", "
        << playerNumber<< std::endl;
    EXPECT_EQ(teamname, "FCPOpp");
    EXPECT_EQ(playerNumber, 1);
  }

  // Test Beam message Parser
  double x,y,z;

  if ( parser->getBeamInformation(parser->socket, x, y, z ) )
  {
    std::cout << "Beam Pos:( "<< x << ", " << y <<", "<< z <<")" <<std::endl;
    EXPECT_DOUBLE_EQ (x, 1);
    EXPECT_DOUBLE_EQ (y, 1);
    EXPECT_DOUBLE_EQ (z, 0.4);
  }


  if (clientThread.joinable())
    clientThread.join();
}
