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


const std::string content = "(scene rsg/agent/nao/nao_hetero.rsg 0)"
      "(beam 1 1 0.4)(he2 -1.80708)(lle1 0)(rle1 0)(lle2 0)(rle2 0)"
      "(lle3 0)(rle3 0)(lle4 0)(rle4 0)(lle5 0)(rle5 0)(lle6 0)(rle6 0)"
      "(lae1 -0.259697)(rae1 -0.259697)(lae2 0)(rae2 0)(lae3 0)(rae3 0)"
      "(lae4 0)(rae4 0)(init (unum 1)(teamname FCPOpp))(he1 3.20802)";

const int kPort = 6334;
std::mutex mutex;
std::condition_variable cv;
bool clientReady = false;
bool serverReady = false;

//////////////////////////////////////////////////
void reset()
{
  clientReady = false;
  serverReady = false;
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
  close(socket);
}

//////////////////////////////////////////////////
TEST(Server, NewClient)
{
  reset();

  auto parser = std::make_shared<ActionMessageParser>();
  gazebo::Server server(kPort, parser,
    &ActionMessageParser::OnConnection, parser.get(),
    &ActionMessageParser::OnDisconnection, parser.get());

  server.Start();
  std::thread clientThread(&senderClient, kPort);

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


  serverReady = true;
  cv.notify_one();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));

  if (clientThread.joinable())
    clientThread.join();
}
