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

#ifndef GAZEBO_ROBOCUP3DS_PERCEPTOR_PLUGIN_HH_
#define GAZEBO_ROBOCUP3DS_PERCEPTOR_PLUGIN_HH_

#include <gazebo/common/UpdateInfo.hh>
#include "robocup3ds/Robocup3dsPlugin.hh"

/// \brief Class to test the communication model.
class PerceptorPlugin : public Robocup3dsPlugin
{
  // Documentation inherited.
  private: virtual void Update(const gazebo::common::UpdateInfo &_info);

  // Number of server iterations executed.
  private: int iterations = 0;
};

#endif
