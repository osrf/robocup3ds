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
#include "robocup3ds/EffectorParser.hh"

/// \brief This test uses s-expression messages belong to a sample RoboCup
/// agent communication.
const std::string content =
    "(scene rsg/agent/nao/nao_hetero.rsg 0)(beam 1 1 0.4)"
    "(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)"
    "(init (unum 1)(teamname sampleAgent))(he1 3.20802)"
    "(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)"
    "(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)";

/// \brief Constants and global variables.
const int kPort = 6234;
std::mutex mutex;
std::condition_variable cv;
bool clientReady = false;
bool serverReady = false;

//////////////////////////////////////////////////
/// \brief Send data and its size in little-endian format to a socket.
/// \param[in] _client_socket Socket for sending/receiving data.
void SendSomeData(int _client_socket)
{
  // Add little-endian, Message size.
  char mBuffer[8192];
  const char *out = reinterpret_cast<const char*> (content.c_str());
  strcpy(mBuffer + 4, out);
  unsigned int len = strlen(out);
  unsigned int netlen = htonl(len);
  memcpy(mBuffer, &netlen, 4);

  std::cout << "msgSent \"" << content << "\"\n\n";
  std::cout << "msglen:" << len << std::endl;

  // Write to socket, and test it.
  size_t sent = write(_client_socket, mBuffer, content.size() + 4);
  fsync(_client_socket);
  EXPECT_EQ(sent, content.size() + 4u);
}

//////////////////////////////////////////////////
/// \brief Create a client and connect it to the server.
/// \param[in] _port Port where the server accepts connections.
/// \param[out] _socket Socket for sending/receiving data to/from the server.
/// \return True when it succeeds or false otherwise.,
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
/// \brief Create a client that connects to the server and wait for a reply.
/// It tests message received by the socket, and  retrieving procedure of
/// the effectors information in s-expression messages.
/// \param[in] _port Port where the server accepts connections.
void receiverClient(const int _port)
{
  int socket;

  if (!createClient(_port, socket))
    return;

  clientReady = true;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cv.notify_one();

  auto parser = std::make_shared<EffectorParser>();

  // Test whether parser receives whole s-expression messages
  // from client socket or not
  EXPECT_TRUE(parser->Parse(socket));

  // Test s-expression messages received by Parse()
  EXPECT_EQ(parser->message.str(), content);

  // Perform information retrieving procedure and update data structures.
  parser->Update();

  // Test Beam effector values
  std::string sceneAddress;
  int rType;

  if ( parser->GetSceneInformation (sceneAddress, rType ) )
  {
    std::cout << "Scene Msg Parsed: " << sceneAddress
        << ", " << rType << std::endl;
    EXPECT_EQ(sceneAddress, "rsg/agent/nao/nao_hetero.rsg");
    EXPECT_EQ(rType, 0);
  }

  // Test Beam effector values
  std::string teamname;
  int playerNumber;

  if ( parser->GetInitInformation(teamname, playerNumber) )
  {
    std::cout << "Init Msg: "<< teamname << ", "
        << playerNumber<< std::endl;
    EXPECT_EQ(teamname, "sampleAgent");
    EXPECT_EQ(playerNumber, 1);
  }

  // Test Beam effector values
  double x, y, z;

  if (parser->GetBeamInformation(x, y, z ))
  {
    std::cout << "Beam Pos:( "<< x << ", " << y <<", "<< z <<")" <<std::endl;
    EXPECT_DOUBLE_EQ(x, 1);
    EXPECT_DOUBLE_EQ(y, 1);
    EXPECT_DOUBLE_EQ(z, 0.4);
  }

  // Test joints effector values
  EXPECT_DOUBLE_EQ(parser->jointEffectors.find("he1")->second, 3.20802);

  EXPECT_DOUBLE_EQ(parser->jointEffectors.find("he2")->second, -1.80708);

  EXPECT_DOUBLE_EQ(parser->jointEffectors.find("lae1")->second, -0.259697);

  EXPECT_DOUBLE_EQ(parser->jointEffectors.find("rae1")->second, -0.259697);

  // Test the interface for joints effectors
  double effector;

  if (parser->GetJointEffector("lae1", effector) )
  {
    EXPECT_DOUBLE_EQ(effector, -0.259697);
  }

  std::cout << "message recieved:" << parser->message.str() << std::endl;
  close(socket);
}

//////////////////////////////////////////////////
/// \brief Test parser procedure and effector information retrieving in one
/// TCP message passing scenario.

TEST(Server, Parser)
{
  // create parser object
  auto parser = std::make_shared <EffectorParser>();

  // create server object with parser object as input
  gazebo::Server server(kPort, parser,
      &EffectorParser::OnConnection, parser.get(),
      &EffectorParser::OnDisconnection, parser.get());


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

  // Send some data to the client using socketID of the connected client.
  SendSomeData(parser->socketID);


  if (clientThread.joinable())
    clientThread.join();
}
