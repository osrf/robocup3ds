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
#include <cstdlib>
#include <map>
#include <memory>
#include <string>
#include <gazebo/common/ModelDatabase.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/physics/JointController.hh>
#include <ignition/math.hh>
#include <sdf/sdf.hh>
#include "robocup3ds/Effector.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/SocketParser.hh"
#include "robocup3ds/Util.hh"

using namespace gazebo;
using namespace common;

int Robocup3dsPlugin::clientPort        = 3100;
int Robocup3dsPlugin::monitorPort       = 3200;
bool Robocup3dsPlugin::syncMode         = false;
const int Robocup3dsPlugin::kBufferSize = 16384;

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin():
  naoSdf(new sdf::Element()),
  gameState(std::make_shared<GameState>()),
  effector(std::make_shared<Effector>(this->gameState.get())),
  monitorEffector(std::make_shared<MonitorEffector>(this->gameState.get())),
  perceptor(std::make_shared<Perceptor>(this->gameState.get())),
  clientServer(std::make_shared<RCPServer>(
                 Robocup3dsPlugin::clientPort,
                 this->effector)),
  monitorServer(std::make_shared<RCPServer>(
                  Robocup3dsPlugin::monitorPort,
                  this->monitorEffector)),
  lastUpdateTime(-GameState::counterCycleTime)
{
  this->buffer = new char[Robocup3dsPlugin::kBufferSize];
  gzmsg << "Starting Robocup Plugin for Gazebo" << std::endl;
}

/////////////////////////////////////////////////
Robocup3dsPlugin::~Robocup3dsPlugin()
{
  delete[] this->buffer;
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::LoadConfiguration(
  const std::map<std::string, std::string> &_config) const
{
  double value;
  bool boolValue;
  if (LoadConfigParameter(_config, "robocup3dsplugin_monitorport", value))
  { Robocup3dsPlugin::monitorPort = static_cast<int>(value); }
  if (LoadConfigParameter(_config, "robocup3dsplugin_clientport", value))
  { Robocup3dsPlugin::clientPort = static_cast<int>(value); }
  if (LoadConfigParameterBool(_config, "robocup3dsplugin_syncmode", boolValue))
  { Robocup3dsPlugin::syncMode = boolValue; }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Load(physics::WorldPtr _world,
                            sdf::ElementPtr _sdf)
{
  // set world sdf and nao sdf
  this->world = _world;
  this->sdf = _sdf;
  std::string filePath = ModelDatabase::
      Instance()->GetModelFile("model://nao_type_zero");
  // gzmsg << "nao filepath: " << filePath << std::endl;
  const sdf::SDFPtr naoSDF(new sdf::SDF());
  sdf::init(naoSDF);
  sdf::readFile(filePath, naoSDF);
  this->naoSdf->Copy(naoSDF->Root());

  // load config parameters
  std::map<std::string, std::string> config;
  _sdf = _sdf->GetFirstElement();
  while (_sdf)
  {
    const auto &param = _sdf->GetValue();
    if (param)
    {
      config[param->GetKey()] = param->GetAsString();
    }
    _sdf = _sdf->GetNextElement();
  }

  gzmsg << "************loading config**************" << std::endl;
  this->gameState->LoadConfiguration(config);
  this->LoadConfiguration(config);
  gzmsg << "************finished loading************" << std::endl;

  gzmsg << "client port: " << Robocup3dsPlugin::clientPort << std::endl;
  gzmsg << "monitor port: " << Robocup3dsPlugin::monitorPort << std::endl;
  gzmsg << "syncmode status: " << Robocup3dsPlugin::syncMode << std::endl;

  // connect to the update event.
  if (!Robocup3dsPlugin::syncMode)
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                               boost::bind(&Robocup3dsPlugin::Update,
                                           this, _1));
    // this->world->GetPhysicsEngine()->SetRealTimeUpdateRate(0.004);
  }
  else
  {
    this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                               boost::bind(&Robocup3dsPlugin::UpdateSync,
                                           this, _1));
    this->world->GetPhysicsEngine()->SetRealTimeUpdateRate(0);
  }

  // start server
  if (this->clientServer->GetPort() != Robocup3dsPlugin::clientPort)
  { this->clientServer->SetPort(Robocup3dsPlugin::clientPort); }
  if (this->monitorServer->GetPort() != Robocup3dsPlugin::monitorPort)
  { this->monitorServer->SetPort(Robocup3dsPlugin::monitorPort); }
  this->clientServer->Start();
  this->monitorServer->Start();
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

  // gzerr << "update() called at " << this->world->GetSimTime().Double()
  //       << std::endl;

  this->UpdateGameState();
  this->UpdatePerceptor();
  this->UpdateEffector();
  this->UpdateMonitorEffector();
  this->lastUpdateTime = this->world->GetSimTime().Double();
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateSync(const common::UpdateInfo & /*_info*/)
{
  this->world->SetPaused(true);
  this->UpdateEffector();
  for (const auto &team : this->gameState->teams)
  {
    for (const auto &agent : team->members)
    {
      if (!agent.syn)
      {
        return;
      }
    }
  }

  this->world->SetPaused(false);
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      agent.syn = false;
    }
  }

  this->UpdateMonitorEffector();
  this->UpdateGameState();
  this->UpdatePerceptor();
  // gzerr << "updatesync() called at " << this->world->GetSimTime().Double()
  //       << std::endl;
  this->lastUpdateTime = this->world->GetSimTime().Double();
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateEffector()
{
  // update effector
  this->effector->Update();

  // insert models into world that need to be inserted
  for (const auto &agentName : this->effector->agentsToAdd)
  {
    sdf::ElementPtr modelRootNode(new sdf::Element());
    modelRootNode->Copy(this->naoSdf);
    const auto &nameAttribute =
      modelRootNode->GetElement("model")->GetAttribute("name");
    nameAttribute->SetFromString(agentName);
    sdf::SDF modelSDF;
    modelSDF.Root(modelRootNode);

    this->world->InsertModelSDF(modelSDF);
    // const auto &model = this->world->GetModel(NaoRobot::defaultModelName);
    // model->SetName(agentName);
  }

  // remove models that need to be removed from world
  for (const auto &agentName : this->effector->agentsToRemove)
  {
    this->world->RemoveModel(agentName);
    gzmsg << "(" << this->world->GetSimTime().Double() <<
          ") agent removed from game world by client: " <<
          agentName << std::endl;
  }

  // set joint velocities of agent model
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      // Get the model of the agent
      const auto &model = this->world->GetModel(agent.GetName());

      // If the agent is not in simulator
      if (!agent.inSimWorld)
      { continue; }


      // Initial the joint values
      if (agent.status == Agent::Status::STOPPED)
      {
        for (const auto &jointName : NaoRobot::hingeJointEffectorMap )
        {
          // Increase the actuators damping to 1
          model->GetJoint(jointName.second)->SetDamping(0, 1);
          model->GetJoint(jointName.second)->SetAngle(0, 0);


          /*
          // Initial the Joint Controller;
          physics::JointControllerPtr jointController(
                            new physics::JointController(model));
          jointController -> Reset();


          jointController -> AddJoint(model->GetJoint(jointName.second));

          // Set the Position PID Values
          jointController->SetPositionPID(
               model->GetJoint(jointName.second)->GetScopedName(),
               common::PID(800, 10, 0.0));

          // Set the joint angle to it origin
          jointController->SetPositionTarget(
                       model->GetJoint(jointName.second)-> GetScopedName(), 0);

          jointController->Update();
        */
        }
        continue;
      }

      // send control the joints, according to joints effectors
      // Joint Effectors are Joints target speed in Radial/Second
      for (auto &kv : agent.action.jointEffectors)
      {
        // Get the Joint name
        std::string naoJointName = NaoRobot::hingeJointEffectorMap.find(
            std::string(kv.first))->second;

        // Initial the Joint Controller;
        physics::JointControllerPtr jointController(
                          new physics::JointController(model));

        jointController -> Reset();

        jointController -> AddJoint(model->GetJoint(naoJointName));

        double targetSpeed = kv.second;

        if (targetSpeed >= 6.13)
        {
          targetSpeed = 6.13;
        }
        else if (targetSpeed <= -6.13)
        {
          targetSpeed = -6.13;
        }
        // Calculate the target degree based on the target Speed
        double target = (kv.second*GameState::counterCycleTime)
            + model->GetJoint(naoJointName)->GetAngle(0).Radian();


        // Set the Position PID Values
        if (naoJointName == "LAnklePitch" || naoJointName == "RAnklePitch")
        {
          jointController->SetPositionPID(
          model->GetJoint(naoJointName)->GetScopedName(),
          common::PID(4000, 4000, 0));
        }
        else if (naoJointName == "LKneePitch" || naoJointName == "RKneePitch")
        {
          jointController->SetPositionPID(
          model->GetJoint(naoJointName)->GetScopedName(),
          common::PID(4000, 4000, 0));
        }
        else if (naoJointName == "LHipPitch" || naoJointName == "RHipPitch")
        {
          jointController->SetPositionPID(
          model->GetJoint(naoJointName)->GetScopedName(),
          common::PID(4000, 4000, 0));
        }
        else
        {
          jointController->SetPositionPID(
          model->GetJoint(naoJointName)->GetScopedName(),
          common::PID(1600, 1600, 0.0));
        }

        // Set the target position
        jointController->SetPositionTarget(
                     model->GetJoint(naoJointName)-> GetScopedName(), target);

        jointController->Update();
      }

      agent.action.jointEffectors.clear();
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateMonitorEffector()
{
  // update monitor effector
  this->monitorEffector->Update();

  // remove models that need to be removed from world
  for (const auto &agentName : this->monitorEffector->agentsToRemove)
  {
    this->world->RemoveModel(agentName);
    gzmsg << "(" << this->world->GetSimTime().Double() <<
          ") agent removed from game world by monitor: " <<
          agentName << std::endl;
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

  const auto &ball = this->world->GetModel(SoccerField::ballName);
  const auto &ballPose = ball->GetWorldPose();
  const auto &ballLink = ball->GetLink(SoccerField::ballLinkName);
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
  // gzmsg << "
  // UpdateGameState()" << std::endl;

  // sync gameState time and gaezbo world time
  this->gameState->SetGameTime(this->world->GetSimTime().Double());

  // use models in gazebo world to update agents and perception info
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      const auto &model = this->world->GetModel(agent.GetName());
      if (model && !agent.inSimWorld)
      {
        agent.inSimWorld = true;
        gzmsg << "(" << this->world->GetSimTime().Double() <<
              ") agent added to game world: " <<
              model->GetName() << std::endl;
      }

      // set agent pose in gameState
      if (agent.updatePose || agent.status == Agent::Status::STOPPED
          || !agent.inSimWorld)
      { continue; }
      const auto &modelPose = model->GetWorldPose();
      agent.pos = G2I(modelPose.pos);
      agent.rot = G2I(modelPose.rot);
    }
  }

  // find ball in gazebo world and use it to update gameState
  const auto &ball = this->world->GetModel(SoccerField::ballName);
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
      if (!agent.inSimWorld)
      { continue; }

      const auto &model = this->world->GetModel(agent.GetName());

      // if agent is in STOPPED state but somehow the model drifts from its
      // gamestate position, it gets moved back
      // const auto &modelPose = model->GetWorldPose();
      // bool correctModelDrift = agent.status == Agent::Status::STOPPED
      //                          && agent.pos.Distance(G2I(modelPose.pos))
      //                          > NaoRobot::torsoHeight;

      if (!agent.updatePose && agent.status != Agent::Status::STOPPED)
      {
        continue;
      }

      ignition::math::Pose3<double> pose(agent.pos, agent.rot);
      model->SetWorldPose(I2G(pose));
      agent.updatePose = false;

      if (agent.status == Agent::Status::STOPPED)
      {
        // reset joint angles, velocity, and acceleration to zero
        // gzmsg << "resetting model!" << std::endl;
        model->ResetPhysicsStates();
        // for (const auto &joint : model->GetJoints())
        // {
        //   joint->Reset();
        // }
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
  // update send information to the agent that sends the Scene message
  for (const auto &socketId : this->effector->sceneMessagesSocketIDs)
  {
    int len = snprintf(this->buffer + 4, Robocup3dsPlugin::kBufferSize - 4,
                       "(time (now %.2f))",
                       this->gameState->GetGameTime());
    unsigned int netlen = htonl(static_cast<unsigned int>(len));
    memcpy(this->buffer, &netlen, 4);
    this->clientServer->Send(socketId, this->buffer, len + 4);
  }

  // update perception related info using gazebo world model
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (!agent.inSimWorld)
      { continue; }

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
    for (const auto &agent : team->members)
    {
      if (!agent.inSimWorld)
      { continue; }

      int cx = perceptor->Serialize(agent, &(this->buffer[4]),
                                    Robocup3dsPlugin::kBufferSize - 4);
      unsigned int _cx = htonl(static_cast<unsigned int>(cx));
      this->buffer[0] =  _cx        & 0xff;
      this->buffer[1] = (_cx >>  8) & 0xff;
      this->buffer[2] = (_cx >> 16) & 0xff;
      this->buffer[3] = (_cx >> 24) & 0xff;
      this->clientServer->Send(agent.socketID, this->buffer, cx + 4);
    }
  }
}
