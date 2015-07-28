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
#include <string>
#include <mutex>
#include <thread>
#include "gtest/gtest.h"

#include "robocup3ds/Effector.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/SocketParser.hh"
#include "robocup3ds/GameState.hh"


/// \brief This test uses s-expression messages belong to a sample RoboCup
/// agent communication.
const std::string message1 =
  "(init (unum 1)(teamname sampleAgent))";

const std::string message2 =
  "(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)""(he1 3.20802)"
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
void SendSomeData(std::string _content, int _client_socket)
{
  // Add little-endian, Message size.
  char mBuffer[16384];
  const char *out = reinterpret_cast<const char *> (_content.c_str());
  snprintf(mBuffer + 4, sizeof(mBuffer) - 4, "%s", out);
  unsigned int len = strlen(out);
  unsigned int netlen = htonl(len);
  memcpy(mBuffer, &netlen, 4);

  std::cout << "msgSent \"" << _content << "\"\n\n";

  // Write to socket, and test it.
  size_t sent = write(_client_socket, mBuffer, _content.size() + 4);
  fsync(_client_socket);
  EXPECT_EQ(sent, _content.size() + 4u);
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

  const int kMaxConnections = 5;
  int tries = 0;
  while (connect(_socket, (struct sockaddr *)&servaddr, sizeof(servaddr)) != 0)
  {
    tries++;
    std::cerr << "createClient::connect() error" << std::endl;
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    if (tries == kMaxConnections)
    { return false; }
  }

  return true;
}

//////////////////////////////////////////////////
/// \brief Create a client that sends some data to the server.
/// \param[in] _port Port where the server accepts connections.
void senderClient(const int _port)
{
  int socket;
  if (!createClient(_port, socket))
  { return; }

  // send the Init Message
  SendSomeData(message1, socket);

  // send the Joints Effectors
  SendSomeData(message2, socket);

  clientReady = true;

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  cv.notify_one();

  // Wait some time until the server checks results.
  {
    std::unique_lock<std::mutex> lk(mutex);
    auto now = std::chrono::system_clock::now();
    if (!cv.wait_until(lk, now + std::chrono::milliseconds(500),
    []() {return serverReady;}))
    {
      FAIL();
    }
  }

  close(socket);
}

// //////////////////////////////////////////////////
// /// \brief Test Parse method and effector information retrieving in one
// /// TCP message passing scenario.
////////////////////////////////////////////////
TEST(RCPServer, Effector)
{
  // create parser object
  auto gameState = std::make_shared<GameState>();
  auto effector = std::make_shared<Effector>(gameState.get());

  // create server object with parser object as input
  RCPServer server(kPort, effector);

  server.Start();
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  std::thread clientThread(&senderClient, kPort);

  // Wait some time until the client sends the request.
  {
    std::unique_lock<std::mutex> lk(mutex);
    auto now = std::chrono::system_clock::now();
    if (!cv.wait_until(lk, now + std::chrono::milliseconds(500),
    []() {return clientReady;}))
    {
      FAIL();
      return;
    }
  }

  serverReady = true;
  cv.notify_one();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  // check the effector information updated in Agent
  for (const auto &team : gameState->teams)
  {
    for (auto &agent : team->members)
    {
      EXPECT_EQ(agent.GetName(), "1_sampleAgent");

      EXPECT_DOUBLE_EQ(agent.action.jointEffectors.find("he1")->second,
                       3.20802);
      EXPECT_DOUBLE_EQ(agent.action.jointEffectors.find("he2")->second,
                       -1.80708);
      EXPECT_DOUBLE_EQ(agent.action.jointEffectors.find("lae1")->second,
                       -0.259697);
      EXPECT_DOUBLE_EQ(agent.action.jointEffectors.find("rae1")->second,
                       -0.259697);
    }
  }

  if (clientThread.joinable())
  { clientThread.join(); }
}
