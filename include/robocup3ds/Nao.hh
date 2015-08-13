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

/// \brief Base Class for Nao body types. All other Nao body types derive
/// from this
class NaoBT
{
  /// \brief Nao constructor
  public: NaoBT() = default;

  /// \brief Nao destructor
  public: ~NaoBT() = default;

  /// \brief Returns height of Nao
  /// \return Height in meters
  public: virtual double KHeight() = 0;

  /// \brief Returns height of Nao's torso
  /// \return Height in meters
  public: virtual double KTorsoHeight() = 0;

  /// \brief Returns a map of link names in Nao model's SDF file and names sent
  /// by server to client
  /// \return Map mentioned above
  public: virtual std::map<std::string, std::string> KBodyPartMap() = 0;

  /// \brief Returns a map of joint names sent received by server from client
  /// and joint names in Nao model's SDF file
  /// \return Map mentioned above
  public: virtual
  std::map<std::string, std::string> HingeJointEffectorMap() = 0;

  /// \brief Returns a map of joint names sent received by server from client
  /// and joint names in Nao model's SDF file
  /// \return Map mentioned above
  public: virtual std::map<std::string, std::string>
  HingeJointPerceptorMap() = 0;

  /// \brief Returns name of link in Nao model that is used for camera position
  /// \return Name mentioned above
  public: virtual std::string KCameraLinkName() = 0;

  /// \brief Returns name of link that is torso
  /// \return Name mentioned above
  public: virtual std::string KTorsoLinkName() = 0;

  /// \brief Returns name of link for left foot
  /// \return Name mentioned above
  public: virtual std::string KLeftFootLinkName() = 0;

  /// \brief Returns name of link for right foot
  /// \return Name mentioned above
  public: virtual std::string KRightFootLinkName() = 0;

  /// \brief Returns default name of Nao model
  /// \return Name mentioned above
  public: virtual std::string KDefaultModelName() = 0;
};

/// \brief The official Nao body type from Aldebaran
class NaoOfficialBT : public NaoBT
{
  // Documentation inherited
  public: virtual double KHeight()
  {
    return this->kHeight;
  }

  // Documentation inherited
  public: virtual double KTorsoHeight()
  {
    return this->kTorsoHeight;
  }

  // Documentation inherited
  public: virtual std::map<std::string, std::string> KBodyPartMap()
  {
    return this->kBodyPartMap;
  }

  // Documentation inherited
  public: virtual
  std::map<std::string, std::string> HingeJointEffectorMap()
  {
    return this->hingeJointEffectorMap;
  }

  // Documentation inherited
  public: virtual std::map<std::string, std::string>
  HingeJointPerceptorMap()
  {
    return this->hingeJointPerceptorMap;
  }

  // Documentation inherited
  public: virtual std::string KCameraLinkName()
  {
    return this->kCameraLinkName;
  }

  // Documentation inherited
  public: virtual std::string KTorsoLinkName()
  {
    return this->kTorsoLinkName;
  }

  // Documentation inherited
  public: virtual std::string KLeftFootLinkName()
  {
    return this->kLeftFootLinkName;
  }

  // Documentation inherited
  public: virtual std::string KRightFootLinkName()
  {
    return this->kRightFootLinkName;
  }

  // Documentation inherited
  public: virtual std::string KDefaultModelName()
  {
    return this->kDefaultModelName;
  }

  // Documentation inherited
  protected: const double kHeight = 0.57;

  // Documentation inherited
  protected: const double kTorsoHeight = kHeight * 0.5;

  // Documentation inherited
  protected: const std::map<std::string, std::string> kBodyPartMap =
  {
    {"head", "Head"},
    {"llowerarm", "LForeArm"},
    {"rlowerarm", "RForeArm"},
    {"lfoot", "LSole"},
    {"rfoot", "RSole"}
  };

  // Documentation inherited
  protected: const
  std::map<std::string, std::string> hingeJointEffectorMap =
  {
    {"he1", "HeadYaw"},
    {"he2", "HeadPitch"},
    {"lae1", "LShoulderPitch"},
    {"lae2", "LShoulderRoll"},
    {"lae3", "LElbowYaw"},
    {"lae4", "LElbowRoll"},
    {"lle1", "LHipYawPitch"},
    {"lle2", "LHipPitch"},
    {"lle3", "LHipRoll"},
    {"lle4", "LKneePitch"},
    {"lle5", "LAnklePitch"},
    {"lle6", "LAnkleRoll"},
    {"rle1", "RHipYawPitch"},
    {"rle2", "RHipPitch"},
    {"rle3", "RHipRoll"},
    {"rle4", "RKneePitch"},
    {"rle5", "RAnklePitch"},
    {"rle6", "RAnkleRoll"},
    {"rae1", "RShoulderPitch"},
    {"rae2", "RShoulderRoll"},
    {"rae3", "RElbowYaw"},
    {"rae4", "RElbowRoll"}
  };

  // Documentation inherited
  protected: const std::map<std::string, std::string>
  hingeJointPerceptorMap =
  {
    {"hj1", "HeadYaw"},
    {"hj2", "HeadPitch"},
    {"laj1", "LShoulderPitch"},
    {"laj2", "LShoulderRoll"},
    {"laj3", "LElbowYaw"},
    {"laj4", "LElbowRoll"},
    {"llj1", "LHipYawPitch"},
    {"llj2", "LHipPitch"},
    {"llj3", "LHipRoll"},
    {"llj4", "LKneePitch"},
    {"llj5", "LAnklePitch"},
    {"llj6", "LAnkleRoll"},
    {"rlj1", "RHipYawPitch"},
    {"rlj2", "RHipPitch"},
    {"rlj3", "RHipRoll"},
    {"rlj4", "RKneePitch"},
    {"rlj5", "RAnklePitch"},
    {"rlj6", "RAnkleRoll"},
    {"raj1", "RShoulderPitch"},
    {"raj2", "RShoulderRoll"},
    {"raj3", "RElbowYaw"},
    {"raj4", "RElbowRoll"}
  };

  // Documentation inherited
  protected: const std::string kCameraLinkName = "Head";

  // Documentation inherited
  protected: const std::string kTorsoLinkName = "Torso";

  // Documentation inherited
  protected: const std::string kLeftFootLinkName = "LSole";

  // Documentation inherited
  protected: const std::string kRightFootLinkName = "RSole";

  // Documentation inherited
  protected: const std::string kDefaultModelName = "naoH25V40";
};

#endif
