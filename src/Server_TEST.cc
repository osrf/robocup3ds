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

/// \brief Constants and global variables.
const std::string content = "hello";
const int kPort = 6234;
std::mutex mutex;
std::condition_variable cv;
bool clientReady = false;
bool serverReady = false;
bool newConnectionDetected = false;
bool newDisconnectionDetected = false;

//////////////////////////////////////////////////
void reset()
{
  clientReady = false;
  serverReady = false;
  newConnectionDetected = false;
  newDisconnectionDetected = false;
}

//////////////////////////////////////////////////
/// \brief This parser assumes that all the messages have a 6 characters.
/// E.g.: "hello\0".
class TrivialSocketParser : public gazebo::SocketParser
{
  /// \brief Class constructor.
  public: TrivialSocketParser() = default;

  /// \brief Class destructor.
  public: ~TrivialSocketParser() = default;

  /// \brief Documentation inherited.
  public: bool Parse(const int _socket)
  {
    char buffer[this->kBufferSize];
    bzero(buffer, sizeof(buffer));
    int bytesRead = 0;
    int totalBytes = 6;

    while (bytesRead < totalBytes)
    {
      int result = recv(_socket, buffer + bytesRead, totalBytes - bytesRead, 0);
      if (result < 1)
      {
        std::cerr << "TrivialSocketParser::Parse() Unable to read" << std::endl;
        return false;
      }

      bytesRead += result;
    }

    EXPECT_EQ(std::string(buffer), content);
    return true;
  }

  public: void OnConnection(const int _socket)
  {
    this->socket = _socket;
    newConnectionDetected = true;
  }

  public: void OnDisconnection(const int /*_socket*/)
  {
    newDisconnectionDetected = true;
  }

  public: void SendSomeData()
  {
    size_t sent = write(this->socket, content.c_str(), content.size() + 1);
    EXPECT_EQ(sent, content.size() + 1u);
  }

  /// \brief Maximum size of each message received.
  private: static const int kBufferSize = 8192;

  /// \brief Socket to the connected client.
  private: int socket;
};

//////////////////////////////////////////////////
/// \brief Create a client and connect it with the server.
/// \param[in] _port Port where the server accepts connections.
/// \param[out] _socket Socket for sending/receiving data to/from the server.
/// \return true when success or false otherwise.,
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
/// \brief Create a client that sends some data to the server.
/// \param[in] _port Port where the server accepts connections.
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
/// \brief Create a client that connects to the server and wait for a reply.
/// \param[in] _port Port where the server accepts connections.
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
  auto parser = std::make_shared<TrivialSocketParser>();
  EXPECT_TRUE(parser->Parse(socket));

  close(socket);
}

//////////////////////////////////////////////////
TEST(Server, Disabled)
{
  reset();

  auto parser = std::make_shared<TrivialSocketParser>();
  gazebo::Server server(kPort, parser,
    &TrivialSocketParser::OnConnection, parser.get(),
    &TrivialSocketParser::OnDisconnection, parser.get());
  EXPECT_FALSE(server.Send(-1, content.c_str(), content.size() + 1));
}

//////////////////////////////////////////////////
TEST(Server, NewClient)
{
  reset();

  EXPECT_FALSE(newConnectionDetected);

  auto parser = std::make_shared<TrivialSocketParser>();
  gazebo::Server server(kPort, parser,
    &TrivialSocketParser::OnConnection, parser.get(),
    &TrivialSocketParser::OnDisconnection, parser.get());

  server.Start();
  std::thread clientThread(&senderClient, kPort);

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
  EXPECT_TRUE(newConnectionDetected);

  serverReady = true;
  cv.notify_one();

  std::this_thread::sleep_for(std::chrono::milliseconds(50));
  // Check that the callback for detecting new disconnections was executed.
  EXPECT_TRUE(newDisconnectionDetected);

  if (clientThread.joinable())
    clientThread.join();
}

////////////////////////////////////////////////
TEST(Server, Send)
{
  reset();

  auto parser = std::make_shared<TrivialSocketParser>();
  gazebo::Server server(kPort + 1, parser,
    &TrivialSocketParser::OnConnection, parser.get(),
    &TrivialSocketParser::OnDisconnection, parser.get());

  server.Start();
  std::thread clientThread(&receiverClient, kPort + 1);

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
  EXPECT_TRUE(newConnectionDetected);

  // Try to send some data using the wrong socket.
  EXPECT_FALSE(server.Send(-1, content.c_str(), content.size() + 1));

  // Send some data to the client.
  parser->SendSomeData();

  if (clientThread.joinable())
    clientThread.join();
}
