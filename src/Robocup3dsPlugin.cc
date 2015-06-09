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
#include <gazebo/gazebo.hh>
#include <sdf/sdf.hh>
#include "robocup3ds/Robocup3dsPlugin.hh"

using namespace gazebo;

const std::string Robocup3dsPlugin::BeforeKickOff   = "BeforeKickOff";
const std::string Robocup3dsPlugin::KickOffLeft     = "KickOff_Left";
const std::string Robocup3dsPlugin::KickOffRight    = "KickOff_Right";
const std::string Robocup3dsPlugin::Play            = "PlayOn";
const std::string Robocup3dsPlugin::KickInLeft      = "KickIn_Left";
const std::string Robocup3dsPlugin::KickInRight     = "KickIn_Right";
const std::string Robocup3dsPlugin::CornerKickLeft  = "corner_kick_left";
const std::string Robocup3dsPlugin::CornerKickRight = "corner_kick_right";
const std::string Robocup3dsPlugin::GoalKickLeft    = "goal_kick_left";
const std::string Robocup3dsPlugin::GoalKickRight   = "goal_kick_right";
const std::string Robocup3dsPlugin::GameOver        = "GameOver";
const std::string Robocup3dsPlugin::GoalLeft        = "Goal_Left";
const std::string Robocup3dsPlugin::GoalRight       = "Goal_Right";
const std::string Robocup3dsPlugin::FreeKickLeft    = "free_kick_left";
const std::string Robocup3dsPlugin::FreeKickRight   = "kick_kick_right";

GZ_REGISTER_WORLD_PLUGIN(Robocup3dsPlugin)

/////////////////////////////////////////////////
Robocup3dsPlugin::Robocup3dsPlugin():
	beforeKickOffState(new BeforeKickOffState(this->BeforeKickOff, this)),
	kickOffLeftState(new KickOffLeftState(this->KickOffLeft, this)),
	kickOffRightState(new KickOffRightState(this->KickOffRight, this)),
	playState(new PlayState(this->Play, this)),
	kickInLeftState(new KickInLeftState(this->KickInLeft, this)),
	kickInRightState(new KickInRightState(this->KickInRight, this)),
	cornerKickLeftState(new CornerKickLeftState(this->CornerKickLeft, this)),
	cornerKickRightState(new CornerKickRightState(this->CornerKickRight, this)),
	goalKickLeftState(new GoalKickLeftState(this->GoalKickLeft, this)),
	goalKickRightState(new GoalKickRightState(this->GoalKickRight, this)),
	gameOverState(new GameOverState(this->GameOver, this)),
	goalLeftState(new GoalLeftState(this->GoalLeft, this)),
	goalRightState(new GoalRightState(this->GoalRight, this)),
	freeKickLeftState(new FreeKickLeftState(this->FreeKickLeft, this)),
	freeKickRightState(new FreeKickRightState(this->FreeKickRight, this))
{
}

/////////////////////////////////////////////////
Robocup3dsPlugin::~Robocup3dsPlugin()
{
}

/////////////////////////////////////////////////
void Robocup3dsPlugin::Load(physics::WorldPtr /*_world*/,
                            sdf::ElementPtr /*_sdf*/)
{
	// Connect to the update event.
	this->updateConnection = event::Events::ConnectWorldUpdateBegin(
	                           boost::bind(&Robocup3dsPlugin::Update, this, _1));
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
