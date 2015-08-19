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

#include <boost/algorithm/string.hpp>
#include <boost/bind.hpp>
#include <boost/shared_ptr.hpp>
#include <netinet/in.h>
#include <algorithm>
#include <cstdlib>
#include <map>
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/Collision.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/Contact.hh>
#include <gazebo/physics/ContactManager.hh>
#include <gazebo/physics/JointController.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/PhysicsEngine.hh>
#include <gazebo/physics/World.hh>
#include <gazebo/transport/TransportTypes.hh>
#include <ignition/math.hh>
#include <string>
#include <sdf/sdf.hh>
#include <vector>

#include "robocup3ds/Effector.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/SocketParser.hh"
#include "robocup3ds/states/State.hh"
#include "robocup3ds/Util.hh"

using namespace gazebo;
using namespace common;
using namespace Util;

int Robocup3dsPlugin::clientPort        = 3100;
int Robocup3dsPlugin::monitorPort       = 3200;
bool Robocup3dsPlugin::syncMode         = false;
const int Robocup3dsPlugin::kBufferSize = 16384;

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin():
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
  lastUpdateTime(-GameState::kCounterCycleTime)
{
  this->buffer = new char[Robocup3dsPlugin::kBufferSize];

  // initialize transport and publisher
  this->gzNode = transport::NodePtr(new transport::Node());
  this->gzNode->Init();
  this->statePub =
    this->gzNode->Advertise<msgs::GzString>("~/robocup3ds/state");
  this->playmodeSub = this->gzNode->Subscribe(
                        "~/robocup3dsGUI/playmode",
                        &Robocup3dsPlugin::UpdateGUIPlaymode, this);
  gzmsg << "Robocup Plugin for Gazebo Started" << std::endl;
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
  {
    Robocup3dsPlugin::monitorPort = static_cast<int>(value);
  }
  if (LoadConfigParameter(_config, "robocup3dsplugin_clientport", value))
  {
    Robocup3dsPlugin::clientPort = static_cast<int>(value);
  }
  if (LoadConfigParameterBool(_config, "robocup3dsplugin_syncmode", boolValue))
  {
    Robocup3dsPlugin::syncMode = boolValue;
  }

  for (const auto &kv : this->gameState->agentBodyTypeMap)
  {
    const auto &bodyType = kv.second;
    std::string bodyTypeName = kv.first;
    boost::algorithm::to_lower(bodyTypeName);
    for (auto &kv2 : bodyType->HingeJointPIDMap())
    {
      std::string jointName = kv2.first;
      boost::algorithm::to_lower(jointName);
      this->LoadPIDParams(kv2.second, bodyTypeName, jointName, _config);
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::LoadPIDParams(common::PID &_pid,
                                     const std::string &_bodyType,
                                     const std::string &_jointName,
                                     const std::map<std::string,
                                     std::string> &_config) const
{
  std::stringstream ss;
  std::vector<double> params;
  try
  {
    const auto &keyName = "pid_" + _bodyType + "_" + _jointName;
    ss << _config.at(keyName);
    double i;
    while (ss >> i)
    {
      params.push_back(i);
      if (ss.peek() == ' ')
      {
        ss.ignore();
      }
    }
  }
  catch (const std::exception &exc)
  {
    return;
  }

  if (params.size() != 5u)
  {
    return;
  }

  gzmsg << "modifying PID params for joint " + _jointName + " in " + _bodyType
        + " bodytype: " << params[0] << " " << params[1] << " " << params[2]
        << " " << params[3] << " " << params[4] << std::endl;
  _pid.SetPGain(params[0]);
  _pid.SetIGain(params[1]);
  _pid.SetDGain(params[2]);
  _pid.SetIMax(params[3]);
  _pid.SetIMin(-params[3]);
  _pid.SetCmdMax(params[4]);
  _pid.SetCmdMin(-params[4]);
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Load(physics::WorldPtr _world,
                            sdf::ElementPtr _sdf)
{
  // set world sdf and nao sdf
  this->world = _world;
  this->sdf = _sdf;

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
  gzmsg << "sync mode status: " << Robocup3dsPlugin::syncMode << std::endl;

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

  // start RCPserver
  if (this->clientServer->GetPort() != Robocup3dsPlugin::clientPort)
  {
    this->clientServer->SetPort(Robocup3dsPlugin::clientPort);
  }
  if (this->monitorServer->GetPort() != Robocup3dsPlugin::monitorPort)
  {
    this->monitorServer->SetPort(Robocup3dsPlugin::monitorPort);
  }
  this->clientServer->Start();
  this->monitorServer->Start();

  // create ball collision filter for contact manager
  const auto &contactMgr = this->world->GetPhysicsEngine()->GetContactManager();
  const auto &ball = this->world->GetModel(SoccerField::kBallName);
  const auto &ballLink = ball->GetLink(SoccerField::kBallLinkName);

  contactMgr->CreateFilter(
    ball->GetName(), ballLink->GetCollision("collision")->GetScopedName());
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Init()
{
  // do nothing
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::PublishGameInfo()
{
  std::string _stateMsg =
    std::to_string(this->gameState->GetElapsedGameTime(true)) +
    " " + this->gameState->GetCurrentState()->name;
  for (const auto &team : this->gameState->teams)
  {
    _stateMsg += "$" + team->name + " (" + Team::GetSideAsString(
                   team->side) + ") (Score: " + std::to_string(team->score)
                 + ") (# of Players: " +
                 std::to_string(team->members.size()) + ")";
  }

  msgs::GzString stateMsg;
  stateMsg.set_data(_stateMsg);
  this->statePub->Publish(stateMsg);
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateGUIPlaymode(ConstGzStringPtr &_msg)
{
  const std::string playModeStr = _msg->data();
  this->gameState->SetCurrent(this->gameState->playModeNameMap[playModeStr]);
  // gzmsg << "GUI changed playmode!" << std::endl;
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  this->UpdateStoppedAgents();
  this->UpdateContactManager();
  // checks if enough time has elapsed to update gameState and send out
  // information
  if (this->world->GetSimTime().Double() - this->lastUpdateTime <
      GameState::kCounterCycleTime)
  {
    return;
  }

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
  this->UpdateMonitorEffector();
  for (const auto &team : this->gameState->teams)
  {
    for (const auto &agent : team->members)
    {
      if (!agent.isSynced)
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
      agent.isSynced = false;
    }
  }

  this->UpdateStoppedAgents();
  this->UpdateContactManager();
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
  for (const auto &agentPtr : this->effector->agentsToAdd)
  {
    std::string path = agentPtr->bodyType->BlueModelPath();
    if (this->gameState->teams.at(1) == agentPtr->team)
    {
      path = agentPtr->bodyType->RedModelPath();
    }

    const sdf::SDFPtr modelSDF(new sdf::SDF());
    sdf::init(modelSDF);
    std::string filePath = ModelDatabase::Instance()->GetModelFile(path);
    sdf::readFile(filePath, modelSDF);

    sdf::ElementPtr modelRootNode = modelSDF->Root();
    const auto &nameAttribute =
      modelRootNode->GetElement("model")->GetAttribute("name");
    nameAttribute->SetFromString(agentPtr->GetName());

    this->world->InsertModelSDF(*modelSDF.get());
  }

  // remove models that need to be removed from world
  for (const auto &agentName : this->effector->agentsToRemove)
  {
    this->world->RemoveModel(agentName);
    gzmsg << "(" << this->world->GetSimTime().Double() <<
          ") agent removed from game world by client: " <<
          agentName << std::endl;
  }

  // disconnect sockets for failed clients
  for (const auto &socketId : this->effector->socketsToDisconnect)
  {
    this->clientServer->DisconnectClient(socketId);
  }

  // set joint velocities of agent model
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (!agent.inSimWorld || agent.status == Agent::Status::STOPPED)
      {
        continue;
      }

      const auto &model = this->world->GetModel(agent.GetName());
      const auto &jointController = model->GetJointController();

      for (auto &kv : agent.action.jointEffectors)
      {
        const auto &joint = model->GetJoint(
                              agent.bodyType->HingeJointEffectorMap()
                              .at(kv.first));
        const auto &scopedJointName = joint->GetScopedName();

        // In simspark the target speed should be in the range
        // of [-6.13, 6.13]
        const double targetSpeed = std::min(std::max(kv.second, -6.13), 6.13);

        const double elapsedTime =
          this->world->GetSimTime().Double() - this->lastUpdateTime;

        // Calculate the target degree based on the target Speed
        const double target = (targetSpeed * elapsedTime)
                              + joint->GetAngle(0).Radian();


        jointController->SetPositionTarget(scopedJointName, target);
      }
      agent.action.jointEffectors.clear();
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::InitJointController(const Agent &_agent,
    const physics::ModelPtr &_model)
{
  const auto &jointController = _model->GetJointController();
  jointController->Reset();
  for (const auto &kv : _agent.bodyType->HingeJointPIDMap())
  {
    const auto &scopedJointName = _model->GetJoint(kv.first)
                                  ->GetScopedName();
    jointController->SetPositionPID(scopedJointName, kv.second);
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
void Robocup3dsPlugin::UpdateContactManager()
{
  const auto &contactMgr = this->world->GetPhysicsEngine()->GetContactManager();
  // gzmsg << "num contacts: " << contactMgr->GetContactCount() << std::endl;
  // gzmsg << "ball position: " << this->world->GetModel(SoccerField::ballName)
  //       ->GetWorldPose().pos << std::endl;
  // const auto &model = this->world->GetModel("1_red");
  // if (model)
  // {
  //   gzmsg << "player position: " << model->GetWorldPose().pos << std::endl;
  // }
  for (unsigned int i = 0; i < contactMgr->GetContactCount(); ++i)
  {
    this->contacts.push_back(*contactMgr->GetContact(i));
  }
  contactMgr->ResetCount();
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

  for (const auto &contact : this->contacts)
  {
    const auto &model1 = contact.collision1->GetModel();
    const auto &model2 = contact.collision2->GetModel();
    if (!model1 || !model2)
    {
      continue;
    }
    physics::ModelPtr ballModel;
    physics::ModelPtr playerModel;
    if (model1->GetName() == SoccerField::kBallName
        && Agent::CheckAgentName(model2->GetName(), uNum, teamName))
    {
      ballModel = model1;
      playerModel = model2;
    }
    else if (Agent::CheckAgentName(model1->GetName(), uNum, teamName)
             && model2->GetName() == SoccerField::kBallName)
    {
      ballModel = model2;
      playerModel = model1;
    }
    else
    {
      continue;
    }

    const auto &ballPose = ballModel->GetWorldPose();
    const auto &lastBallContact = gameState->GetLastBallContact();

    // only update the last ball contact if contact by same agent
    // has occurred less than a certain time interval ago, otherwise
    // add new ball contact to history
    if (lastBallContact
        && lastBallContact->uNum == uNum
        && lastBallContact->side == teamSide[teamName]
        && gameState->GetGameTime() - lastBallContact->contactTime
        < GameState::kBallContactInterval
        && gameState->GetCurrentState()->name == lastBallContact->playMode)
    {
      lastBallContact->contactTime = gameState->GetGameTime();
      break;
      // gzmsg << "last ball contact updated: " << playerModel->GetName() <<
      //       " " << gameState->GetGameTime() << " " <<
      //       lastBallContact->contactTime << std::endl;
    }
    else
    {
      std::shared_ptr<GameState::BallContact> ballContact(
        new GameState::BallContact(uNum, teamSide[teamName],
                                   gameState->GetGameTime(),
                                   G2I(ballPose.pos),
                                   gameState->GetCurrentState()->name));
      gameState->ballContactHistory.push_back(ballContact);
      break;
      // gzmsg << "new ball contact: " << playerModel->GetName() <<
      //       " " << gameState->GetGameTime() << std::endl;
      // gzmsg << "total number of contacts: " <<
      //       this->gameState->ballContactHistory.size() << std::endl;
    }
  }
  this->contacts.clear();
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdateGameState()
{
  // gzmsg << "UpdateGameState()" << std::endl;

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
        this->InitJointController(agent, model);
        gzmsg << "(" << this->world->GetSimTime().Double() <<
              ") agent added to game world: " <<
              model->GetName() << std::endl;
      }

      // set agent pose in gameState
      if (agent.updatePose || agent.status == Agent::Status::STOPPED
          || !agent.inSimWorld)
      {
        continue;
      }
      const auto &modelPose = model->GetWorldPose();
      agent.pos = G2I(modelPose.pos);
      agent.rot = G2I(modelPose.rot);
    }
  }

  // find ball in gazebo world and use it to update gameState
  const auto &ball = this->world->GetModel(SoccerField::kBallName);
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
      if (!agent.inSimWorld || !agent.updatePose)
      {
        continue;
      }
      const auto &model = this->world->GetModel(agent.GetName());

      ignition::math::Pose3<double> pose(agent.pos, agent.rot);
      model->SetWorldPose(I2G(pose));
      agent.updatePose = false;
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
void Robocup3dsPlugin::UpdateStoppedAgents()
{
  for (const auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      if (!agent.inSimWorld || agent.status != Agent::Status::STOPPED)
      {
        continue;
      }
      const auto &model = this->world->GetModel(agent.GetName());

      model->GetJointController()->Reset();
      for (const auto &joint : model->GetJoints())
      {
        joint->Reset();
      }
      model->ResetPhysicsStates();
      this->gameState->MoveAgent(agent, agent.pos.X(), agent.pos.Y(),
                                 agent.rot.Euler().Z());
      ignition::math::Pose3<double> pose(agent.pos, agent.rot);
      model->SetWorldPose(I2G(pose));
      agent.updatePose = false;
    }
  }
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::UpdatePerceptor()
{
  // update send information to the agent that sends the Scene message
  for (const auto &kv : this->effector->socketIDbodyTypeMap)
  {
    const int socketId = kv.first;
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
      {
        continue;
      }

      const auto &model = this->world->GetModel(agent.GetName());

      // update agent's camera pose
      auto &cameraPose = model->GetLink(agent.bodyType->CameraLinkName())->
                         GetWorldPose();
      agent.cameraPos = G2I(cameraPose.pos);
      agent.cameraRot = G2I(cameraPose.rot);

      // update agent's self body map
      for (auto &kv : agent.bodyType->BodyPartMap())
      {
        agent.selfBodyMap[kv.first] =
          G2I(model->GetLink(kv.second)->GetWorldPose().pos);
      }
      // update agent's percept joints angles
      for (auto &kv : agent.bodyType->HingeJointPerceptorMap())
      {
        agent.percept.hingeJoints[kv.first] =
          model->GetJoint(kv.second)->GetAngle(0).Degree();
      }

      // update agent's percept gyro rate
      const auto &torsoLink = model->GetLink(agent.bodyType->TorsoLinkName());
      agent.percept.gyroRate = G2I(torsoLink->GetWorldAngularVel());

      // update agent's percept acceleration
      agent.percept.accel = G2I(torsoLink->GetWorldLinearAccel());

      // update agent's percept left and right foot force info
      agent.percept.leftFootFR =
        std::make_pair(
          G2I(model->GetLink(agent.bodyType->LeftFootLinkName())->
              GetWorldPose().pos),
          G2I(model->GetLink(agent.bodyType->LeftFootLinkName())->
              GetWorldForce()));

      agent.percept.rightFootFR =
        std::make_pair(
          G2I(model->GetLink(agent.bodyType->RightFootLinkName())->
              GetWorldPose().pos),
          G2I(model->GetLink(agent.bodyType->RightFootLinkName())->
              GetWorldForce()));
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
      {
        continue;
      }

      int cx = perceptor->Serialize(agent, &(this->buffer[4]),
                                    Robocup3dsPlugin::kBufferSize - 4);
      unsigned int _cx = htonl(static_cast<unsigned int>(cx));
      memcpy(this->buffer, &_cx, 4);
      this->clientServer->Send(agent.socketID, this->buffer, cx + 4);
    }
  }

  // publish game information to gui plugin
  this->PublishGameInfo();
}
