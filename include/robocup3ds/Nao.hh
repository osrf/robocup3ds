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

#ifndef _GAZEBO_NAO_HH_
#define _GAZEBO_NAO_HH_

#include <string>
#include <vector>

namespace NaoRobot
{
  static const double height = 0.6;

  static const double torsoHeight = 0.3;

  static const std::vector<std::string> bodyParts =
  {
    "LForeArm",
    "RForeArm",
    "l_ankle",
    "r_ankle",
    "Head"
  };

  static const std::string cameraLinkName = "CameraTop_joint";

  static const std::string torsoLinkName = "torso";

  static const std::string leftFootLinkName = "l_sole";

  static const std::string rightFootLinkName = "r_sole";
}

#endif
