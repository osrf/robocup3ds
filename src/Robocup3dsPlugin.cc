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
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/World.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>

#include "robocup3ds/Effectors.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/Util.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin()
{
  this->server = new RCPServer();
  this->effector = new Effector(this->gameState);
  this->gameState = new GameState();
  this->perceptor = new Perceptor(this->gameState, this->server);

  this->server->Start();
}

/////////////////////////////////////////////////
Robocup3dsPlugin::~Robocup3dsPlugin()
{
  delete this->server;
  delete this->effector;
  delete this->gameState;
  delete this->perceptor;
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
  this->UpdateEffector();
  this->UpdateGameState();
  this->UpdatePerceptor();
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateEffector()
{
  this->effector->Update();
  this->world->InsertModelFile("model://nao");
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateGameState()
{
  // use models in gazebo world to update gameState
  for (auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      // set camera and body pose in gameState
      physics::ModelPtr model = this->world->GetModel(agent.GetName());
      auto &modelPose = model->GetWorldPose();
      auto &cameraPose = model->GetLink("CameraTop_joint")->GetWorldPose();
      agent.cameraPos = G2I(cameraPose.pos);
      agent.cameraRot = G2I(cameraPose.rot);
      agent.pos = G2I(modelPose.pos);
      agent.rot = G2I(modelPose.rot);
      agent.updatePose = false;

      // set bodyMap in gameState
    }
  }
  // find ball in gazebo world and use it to update gameState
  physics::ModelPtr ball = this->world->GetModel("ball");
  auto &ballPose = ball->GetWorldPose();
  this->gameState->MoveBall(G2I(ballPose.pos));
  this->gameState->SetBallVel(G2I(ball->GetWorldLinearVel()));
  this->gameState->SetBallAngVel(G2I(ball->GetWorldAngularVel()));
  this->gameState->updateBallPose = false;

  // update game state
  this->gameState->Update();

  // use gameState agents to update gazebo world models
  for (auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (!agent.updatePose)
      { continue; }
      physics::ModelPtr model = this->world->GetModel(agent.GetName());
      ignition::math::Pose3<double> pose(agent.pos, agent.rot);
      model->SetWorldPose(I2G(pose));
      agent.updatePose = false;
    }
  }
  // use gameState ball to update gazebo world
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
  this->perceptor->Update();
  this->perceptor->SendToServer();
}
