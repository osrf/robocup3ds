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
    gzmsg << "-- Checking that robot is at rest" << std::endl;

    // Aceelerometer registers gravity at rest
    EXPECT_NEAR(agent->percept.accel.X(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.accel.Y(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.accel.Z(), 9.8, 0.4);

    // Gyro registers no velocity at rest
    EXPECT_NEAR(agent->percept.gyroRate.X(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.gyroRate.Y(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.gyroRate.Z(), 0.0, 0.4);
  }
  else if (this->iterations == 2000)
  {
    gzmsg << "-- Dropping robot" << std::endl;

    auto model = this->world->GetModel(agent->GetName());
    ASSERT_TRUE(model != NULL);
    // Force a free fall to generate some random angular velocity.
    model->SetWorldPose(gazebo::math::Pose(0, 0, 50, 0, 0, 0));
  }
  else if (this->iterations == 2500)
  {
    gzmsg << "-- Checking that robot is at free fall" << std::endl;

    // Accelerometer registers no acceleration on free-fall
    EXPECT_NEAR(agent->percept.accel.X(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.accel.Y(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.accel.Z(), 0.0, 0.4);

    // Check that robot is spinning due to its uneven inertia
    EXPECT_FALSE(ignition::math::equal(agent->percept.gyroRate.X(), 0.0));
    EXPECT_FALSE(ignition::math::equal(agent->percept.gyroRate.Y(), 0.0));
    EXPECT_FALSE(ignition::math::equal(agent->percept.gyroRate.Z(), 0.0));
  }
  else if (this->iterations == 3000)
  {
    gzmsg << "-- Laying down the robot" << std::endl;

    auto model = this->world->GetModel(agent->GetName());
    ASSERT_TRUE(model != NULL);
    // Force the model to lie on its back
    model->SetWorldPose(gazebo::math::Pose(0, 0, 0.4, 3.14, -1.57, 3.14));
  }
  else if (this->iterations == 4000)
  {
    gzmsg << "-- Checking robot is laid down" << std::endl;

    // Accelerometer registers gravity mostly on torso's X axis
    EXPECT_NEAR(agent->percept.accel.X(), -9.8, 3.0);
    EXPECT_NEAR(agent->percept.accel.Y(), 0.0, 3.0);
    EXPECT_NEAR(agent->percept.accel.Z(), 0.0, 3.0);

    // Gyro registers no velocity at rest
    EXPECT_NEAR(agent->percept.gyroRate.X(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.gyroRate.Y(), 0.0, 0.4);
    EXPECT_NEAR(agent->percept.gyroRate.Z(), 0.0, 0.4);
  }
}
