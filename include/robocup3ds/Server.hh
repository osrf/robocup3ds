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

#ifndef INCLUDE_ROBOCUP3DS_SERVER_HH_
#define INCLUDE_ROBOCUP3DS_SERVER_HH_

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <time.h>
#include <string.h>
#include <sstream>
#include <thread>

using namespace std;

typedef struct {
    int sock;
    struct sockaddr address;
    socklen_t addr_len;
} connection_t;

class Server {
    static Server* uniqueInstance;
    int clientCounter, agentNo;
    stringstream sendingMessage;
    std::string receivingMessage;

public:

    Server();

    static Server* getUniqueInstance();

    std::string getSendingMessage();

    std::string getRecievingMessage();

    int getAgentNo();

    void messagePassing(int sock);

    void start();

    virtual ~Server();

    void error(const char *msg) {
        perror(msg);
        exit(1);
    }
};
#endif /* INCLUDE_ROBOCUP3DS_SERVER_HH_ */
