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

#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <netinet/in.h>
#include <map>
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math.hh>
#include <string>
#include <sdf/sdf.hh>

#include "robocup3ds/Effectors.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/Util.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin():
  lastUpdateTime(-GameState::counterCycleTime)
{
  this->server = new RCPServer();
  this->effector = new Effector(this->gameState);
  this->gameState = new GameState();
  this->perceptor = new Perceptor(this->gameState);
  this->buffer = new char[Robocup3dsPlugin::kBufferSize];

  this->server->Start();
}

/////////////////////////////////////////////////
Robocup3dsPlugin::~Robocup3dsPlugin()
{
  delete this->server;
  delete this->effector;
  delete this->gameState;
  delete this->perceptor;
  delete[] this->buffer;
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Load(physics::WorldPtr _world,
                            sdf::ElementPtr _sdf)
{
  // Connect to the update event.
  this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                             boost::bind(&Robocup3dsPlugin::Update, this, _1));
  this->world = _world;
  this->sdf = _sdf;
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Init()
{
  // do nothing
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  // checks if enough time has elapsed to update gameState and send out
  // information
  if (this->world->GetSimTime().Double() - this->lastUpdateTime <
      GameState::counterCycleTime)
  {
    return;
  }

  this->UpdateEffector();
  this->UpdateGameState();
  this->UpdatePerceptor();
  this->lastUpdateTime = this->world->GetSimTime().Double();
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateEffector()
{
  // update effector
  this->effector->Update();

  // insert models into world that need to be inserted
  for (auto &agentInfo : this->effector->agentsToAdd)
  {
    std::string agentName = std::to_string(agentInfo.uNum) + "_" +
                            agentInfo.teamName;
    this->world->InsertModelFile("model://nao");
    auto model = this->world->GetModel(NaoRobot::defaultModelName);
    model->SetName(agentName);
  }

  // remove models that need to be removed from world
  for (auto &agentInfo : this->effector->agentsToRemove)
  {
    std::string agentName = std::to_string(agentInfo.uNum) + "_" +
                            agentInfo.teamName;
    this->world->RemoveModel(agentName);
  }

  // set joint velocities of agent model
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (agent.status == Agent::Status::STOPPED)
      { continue; }
      auto model = this->world->GetModel(agent.GetName());
      for (auto &kv : agent.action.jointEffectors)
      {
        model->GetJoint(kv.first)->SetVelocity(0, kv.second);
      }
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateBallContactHistory()
{
  std::map<std::string, Team::Side> teamSide;
  for (const auto &team : this->gameState->teams)
  {
    teamSide[team->name] = team->side;
  }
  int uNum;
  std::string teamName;

  const auto &ball = this->world->GetModel("ball_model");
  const auto &ballPose = ball->GetWorldPose();
  const auto &ballLink = ball->GetLink("ball_link");
  for (const auto &collision : ballLink->GetCollisions())
  {
    const auto &model = collision->GetModel();
    // make sure that model belongs to an agent
    if (model && Agent::CheckAgentName(model->GetName(), uNum, teamName))
    {
      const auto &lastBallContact = gameState->GetLastBallContact();

      // only update the last ball contact if contact by same agent
      // has occurred less than a certain time interval ago, otherwise
      // add new ball contact to history
      if (lastBallContact
          && lastBallContact->uNum == uNum
          && lastBallContact->side == teamSide[teamName]
          && gameState->GetGameTime() - lastBallContact->contactTime
          < GameState::ballContactInterval)
      {
        lastBallContact->contactTime = gameState->GetGameTime();
      }
      else
      {
        std::shared_ptr<GameState::BallContact> ballContact(
          new GameState::BallContact(uNum, teamSide[teamName],
                                     gameState->GetGameTime(),
                                     G2I(ballPose.pos)));
        gameState->ballContactHistory.push_back(ballContact);
      }
      break;
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateGameState()
{
  // sync gameState time and gaezbo world time
  this->gameState->SetGameTime(this->world->GetSimTime().Double());

  // use models in gazebo world to update agents and perception info
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      // set agent pose in gameState
      if (agent.updatePose)
      { continue; }
      const auto &model = this->world->GetModel(agent.GetName());
      auto &modelPose = model->GetWorldPose();
      agent.pos = G2I(modelPose.pos);
      agent.rot = G2I(modelPose.rot);
    }
  }

  // find ball in gazebo world and use it to update gameState
  const auto &ball = this->world->GetModel("ball_model");
  auto &ballPose = ball->GetWorldPose();
  if (!this->gameState->updateBallPose)
  {
    this->gameState->MoveBall(G2I(ballPose.pos));
    this->gameState->SetBallVel(G2I(ball->GetWorldLinearVel()));
    this->gameState->SetBallAngVel(G2I(ball->GetWorldAngularVel()));
  }

  // update ball contact history
  this->UpdateBallContactHistory();

  // update game state
  this->gameState->Update();

  // use gameState agents pose to update gazebo world agent pose
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (!agent.updatePose)
      { continue; }
      const auto &model = this->world->GetModel(agent.GetName());
      ignition::math::Pose3<double> pose(agent.pos, agent.rot);
      model->SetWorldPose(I2G(pose));
      agent.updatePose = false;

      if (agent.status == Agent::Status::STOPPED
          && agent.prevStatus == Agent::Status::RELEASED)
      {
        // reset joint angles, velocity, and acceleration to zero,
        // restrict movement
      }
      else if (agent.status == Agent::Status::RELEASED
               && agent.prevStatus == Agent::Status::STOPPED)
      {
        // allow joints to move again
      }
    }
  }

  // use gameState ball to update gazebo world ball
  if (this->gameState->updateBallPose)
  {
    auto newBallPose =
      math::Pose(I2G(this->gameState->GetBall()), ballPose.rot);
    ball->SetWorldPose(newBallPose);
    ball->SetAngularVel(I2G(this->gameState->GetBallAngVel()));
    ball->SetLinearVel(I2G(this->gameState->GetBallVel()));
    this->gameState->updateBallPose = false;
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdatePerceptor()
{
  // update perception related info using gazebo world model
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      const auto &model = this->world->GetModel(agent.GetName());

      // update agent's camera pose
      auto &cameraPose = model->GetLink(NaoRobot::cameraLinkName)->
                         GetWorldPose();
      agent.cameraPos = G2I(cameraPose.pos);
      agent.cameraRot = G2I(cameraPose.rot);

      // update agent's self body map
      for (auto &kv : NaoRobot::bodyPartMap)
      {
        agent.selfBodyMap[kv.first] =
          G2I(model->GetLink(kv.second)->GetWorldPose().pos);
      }

      // update agent's percept joints angles
      for (auto &kv : NaoRobot::hingeJointPerceptorMap)
      {
        agent.percept.hingeJoints[kv.first] =
          model->GetJoint(kv.second)->GetAngle(0).Degree();
      }

      // update agent's percept gyro rate
      const auto &torsoLink = model->GetLink(NaoRobot::torsoLinkName);
      agent.percept.gyroRate = G2I(torsoLink->GetWorldAngularVel());

      // update agent's percept acceleration
      agent.percept.accel = G2I(torsoLink->GetWorldLinearAccel());

      // update agent's percept left and right foot force info
      agent.percept.leftFootFR =
        std::make_pair(
          G2I(model->GetLink(NaoRobot::leftFootLinkName)->GetWorldPose().pos),
          G2I(model->GetLink(NaoRobot::leftFootLinkName)->GetWorldForce()));

      agent.percept.rightFootFR =
        std::make_pair(
          G2I(model->GetLink(NaoRobot::rightFootLinkName)->GetWorldPose().pos),
          G2I(model->GetLink(NaoRobot::rightFootLinkName)->GetWorldForce()));
    }
  }
  // call update function
  this->perceptor->Update();

  // send messages to server
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      int cx = perceptor->Serialize(agent, &(this->buffer[4]),
                                    Robocup3dsPlugin::kBufferSize - 4);
      unsigned int _cx = htonl(static_cast<unsigned int>(cx));
      this->buffer[0] =  _cx        & 0xff;
      this->buffer[1] = (_cx >>  8) & 0xff;
      this->buffer[2] = (_cx >> 16) & 0xff;
      this->buffer[3] = (_cx >> 24) & 0xff;
      server->Send(agent.socketID, this->buffer, cx + 4);
    }
  }
}
