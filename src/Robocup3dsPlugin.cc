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
#include <memory>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/World.hh>
#include <sdf/sdf.hh>

#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Effectors.hh"
#include "robocup3ds/Perceptors.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin():
  gameState(std::make_shared<GameState>())
{
  perceptor = std::make_shared<Perceptor>(gameState);
  effector = std::make_shared<Effector>(gameState);
}

/////////////////////////////////////////////////
Robocup3dsPlugin::~Robocup3dsPlugin()
{
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

  this->world->InsertModelFile("model://nao");
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Init()
{
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Update(const common::UpdateInfo & /*_info*/)
{
  // std::cout << "Update" << std::endl;
}
