/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

#include <string>
#include <gazebo/physics/Model.hh>
#include "robocup3ds/Robocup3dsPlugin.hh"
#include "robocup3ds/states/CornerKickRightState.hh"
#include "robocup3ds/SoccerField.hh"

using namespace gazebo;

/////////////////////////////////////////////////
CornerKickRightState::CornerKickRightState(const std::string &_name,
                         Robocup3dsPlugin *_plugin)
  : State(_name, _plugin)
{
}

/////////////////////////////////////////////////
void CornerKickRightState::Initialize()
{
  State::Initialize();

  // Get the position of the ball in the field reference frame.
  math::Pose ballPose = this->plugin->GetBall();

  // Move the ball to the corner.
  this->plugin->MoveBall(math::Pose(
    (fabs(ballPose.pos.x) / ballPose.pos.x) * SoccerField::HalfFieldHeight,
    (fabs(ballPose.pos.y) / ballPose.pos.y) * SoccerField::HalfFieldWidth,
    ballPose.pos.z, 0, 0, 0));
}

/////////////////////////////////////////////////
void CornerKickRightState::Update()
{
  // The left team is not allowed to be close to the ball.
  this->plugin->DropBallImpl(1);

  // After some time, go to play mode.
  common::Time elapsed = this->timer.GetElapsed();
  if (elapsed.sec > 5)
    this->plugin->SetCurrent(this->plugin->playState.get());
}
