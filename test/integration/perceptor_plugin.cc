/*
 * Copyright (C) 2016 Open Source Robotics Foundation
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

#include <gtest/gtest.h>
#include <gazebo/math/Pose.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>
#include "robocup3ds/GameState.hh"
#include "perceptor_plugin.hh"

using namespace gazebo;

GZ_REGISTER_WORLD_PLUGIN(PerceptorPlugin)

//////////////////////////////////////////////////
void PerceptorPlugin::Update(const gazebo::common::UpdateInfo &_info)
{
  Robocup3dsPlugin::Update(_info);

  ++this->iterations;

  std::shared_ptr<Team> team = this->gameState->teams.at(0);
  Agent *agent = nullptr;

  if (this->iterations >= 1000)
  {
    agent = &(team->members.at(0));
    ASSERT_TRUE(agent != NULL);
  }

  if (this->iterations == 1000)
  {
    EXPECT_NEAR(agent->percept.accel.Z(), 9.8, 0.4);
  }
  else if (this->iterations == 2000)
  {
    auto model = this->world->GetModel(agent->GetName());
    ASSERT_TRUE(model != NULL);
    // Force a free fall to generate some random angular velocity.
    model->SetWorldPose(gazebo::math::Pose(0, 0, 50, 0, 0, 0));
  }
  else if (this->iterations == 2500)
  {
    EXPECT_FALSE(ignition::math::equal(agent->percept.gyroRate.Y(), 0.0));
  }
}
