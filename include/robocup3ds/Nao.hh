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

#ifndef _GAZEBO_ROBOCUP3DS_NAO_HH_
#define _GAZEBO_ROBOCUP3DS_NAO_HH_

#include <string>
#include <map>
#include <vector>

namespace NaoRobot
{
  /// \brief Height of Nao robot in meters
  static const double kHeight = 0.6;

  /// \brief Height of Nao robot's torso in meters
  static const double kTorsoHeight = 0.3;

  /// \brief A map of link names in Nao model's SDF file and names sent
  /// by server to client
  static const std::map<std::string, std::string> kBodyPartMap =
  {
    {"head", "Head"},
    {"llowerarm", "LForeArm"},
    {"rlowerarm", "RForeArm"},
    {"lfoot", "l_sole"},
    {"rfoot", "r_sole"}
  };

  /// \brief A map of joint names sent received by server from client and
  /// joint names in Nao model's SDF file
  static const std::map<std::string, std::string> kHingeJointEffectorMap =
  {
    {"he1", "HeadYaw"},
    {"he2", "HeadPitch"},
    {"lae1", "LShoulderPitch"},
    {"lae2", "LShoulderRoll"},
    {"lae3", "LElbowRoll"},
    {"lae4", "LElbowYaw"},
    {"lle1", "LHipYawPitch"},
    {"lle2", "LHipRoll"},
    {"lle3", "LHipPitch"},
    {"lle4", "LKneePitch"},
    {"lle5", "LAnklePitch"},
    {"lle6", "LAnkleRoll"},
    {"rle1", "RHipYawPitch"},
    {"rle2", "RHipRoll"},
    {"rle3", "RHipPitch"},
    {"rle4", "RKneePitch"},
    {"rle5", "RAnklePitch"},
    {"rle6", "RAnkleRoll"},
    {"rae1", "RShoulderPitch"},
    {"rae2", "RShoulderRoll"},
    {"rae3", "RElbowRoll"},
    {"rae4", "RElbowYaw"}
  };

  /// \brief A map of joint names sent received by server from client and
  /// joint names in Nao model's SDF file
  static const std::map<std::string, std::string> kHingeJointPerceptorMap =
  {
    {"hj1", "HeadYaw"},
    {"hj2", "HeadPitch"},
    {"laj1", "LShoulderPitch"},
    {"laj2", "LShoulderRoll"},
    {"laj3", "LElbowRoll"},
    {"laj4", "LElbowYaw"},
    {"llj1", "LHipYawPitch"},
    {"llj2", "LHipRoll"},
    {"llj3", "LHipPitch"},
    {"llj4", "LKneePitch"},
    {"llj5", "LAnklePitch"},
    {"llj6", "LAnkleRoll"},
    {"rlj1", "RHipYawPitch"},
    {"rlj2", "RHipRoll"},
    {"rlj3", "RHipPitch"},
    {"rlj4", "RKneePitch"},
    {"rlj5", "RAnklePitch"},
    {"rlj6", "RAnkleRoll"},
    {"raj1", "RShoulderPitch"},
    {"raj2", "RShoulderRoll"},
    {"raj3", "RElbowRoll"},
    {"raj4", "RElbowYaw"}
  };

  /// \brief Name of link in Nao model that is used for camera position
  static const std::string kCameraLinkName = "Head";

  /// \brief Name of link that is torso
  static const std::string kTorsoLinkName = "base_link";

  /// \brief Name of link for left foot
  static const std::string kLeftFootLinkName = "l_sole";

  /// \brief Name of link for right foot
  static const std::string kRightFootLinkName = "r_sole";

  /// \brief Default name of Nao model
  static const std::string kDefaultModelName = "naoH25V40";
}

#endif
