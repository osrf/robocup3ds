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

#include <gazebo/common/PID.hh>
#include <map>
#include <memory>
#include <string>
#include <vector>

/// \brief Base Class for Nao body types. All other Nao body types derive
/// from this. By default and unless specified otherwise, the body type used
// is NaoOfficialBT
class NaoBT
{
  /// \brief Nao constructor
  public: NaoBT() = default;

  /// \brief Nao destructor
  public: ~NaoBT() = default;

  /// \brief Returns height of Nao
  /// \return Height in meters
  public: virtual double Height() const = 0;

  /// \brief Returns height of Nao's torso
  /// \return Height in meters
  public: virtual double TorsoHeight() const = 0;

  /// \brief Returns a map of link names in Nao model's SDF file and names sent
  /// by server to client
  /// \return Map mentioned above
  public: virtual const
  std::map<std::string, std::string>& BodyPartMap() const = 0;

  /// \brief Returns a map of joint names sent received by server from client
  /// and joint names in Nao model's SDF file
  /// \return Map mentioned above
  public: virtual const
  std::map<std::string, std::string>& HingeJointEffectorMap() const = 0;

  /// \brief Returns a map of joint names and their corresponding PIDs
  /// \return Map mentioned above
  public: virtual std::map<std::string, gazebo::common::PID>&
  HingeJointPIDMap() = 0;

  /// \brief Returns a map of joint names sent received by server from client
  /// and joint names in Nao model's SDF file
  /// \return Map mentioned above
  public: virtual const std::map<std::string, std::string>&
  HingeJointPerceptorMap() const = 0;

  /// \brief Returns name of link in Nao model that is used for camera position
  /// \return Name mentioned above
  public: virtual std::string CameraLinkName() const = 0;

  /// \brief Returns name of link that is torso
  /// \return Name mentioned above
  public: virtual std::string TorsoLinkName() const = 0;

  /// \brief Returns name of link for left foot
  /// \return Name mentioned above
  public: virtual std::string LeftFootLinkName() const = 0;

  /// \brief Returns name of link for right foot
  /// \return Name mentioned above
  public: virtual std::string RightFootLinkName() const = 0;

  /// \brief Returns default name of Nao model
  /// \return Name mentioned above
  public: virtual std::string DefaultModelName() const = 0;

  /// \brief Returns URI of blue model used by this body type
  /// \return URI mentioned above
  public: virtual std::string BlueModelPath() const = 0;

  /// \brief Returns URI of red model used by this body type
  /// \return URI mentioned above
  public: virtual std::string RedModelPath() const = 0;
};

/// \brief The official Nao body type from Aldebaran
class NaoOfficialBT : public NaoBT
{
  // Documentation inherited
  public: virtual double Height() const
  {
    return 0.6;
  }

  // Documentation inherited
  public: virtual double TorsoHeight() const
  {
    return this->Height() * 0.5;
  }

  // Documentation inherited
  public: virtual const std::map<std::string, std::string>& BodyPartMap() const
  {
    return this->kBodyPartMap;
  }

  // Documentation inherited
  public: virtual
  const std::map<std::string, std::string>& HingeJointEffectorMap() const
  {
    return this->hingeJointEffectorMap;
  }

  // Documentation inherited
  public: virtual const std::map<std::string, std::string>&
  HingeJointPerceptorMap() const
  {
    return this->hingeJointPerceptorMap;
  }

  // Documentation inherited
  public: virtual std::map<std::string, gazebo::common::PID>&
  HingeJointPIDMap()
  {
    return this->hingeJointPIDMap;
  }

  // Documentation inherited
  public: virtual std::string CameraLinkName() const
  {
    return "Head";
  }

  // Documentation inherited
  public: virtual std::string TorsoLinkName() const
  {
    return "Torso";
  }

  // Documentation inherited
  public: virtual std::string LeftFootLinkName() const
  {
    return "LSole";
  }

  // Documentation inherited
  public: virtual std::string RightFootLinkName() const
  {
    return "RSole";
  }

  // Documentation inherited
  public: virtual std::string DefaultModelName() const
  {
    return "naoH25V40";
  }

  // Documentation inherited
  public: virtual std::string BlueModelPath() const
  {
    return "model://nao_blue";
  }

  // Documentation inherited
  public: virtual std::string RedModelPath() const
  {
    return "model://nao_red";
  }

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
    {"raj3", "RElbowYaw"},
    {"raj4", "RElbowRoll"}
  };

  // documentation inherited
  protected: std::map<std::string, gazebo::common::PID>
  hingeJointPIDMap =
  {
    {"HeadYaw", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"HeadPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LShoulderPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LShoulderRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LElbowYaw", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LElbowRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LHipYawPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LHipRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LHipPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LKneePitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LAnklePitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"LAnkleRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RHipYawPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RHipRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RHipPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RKneePitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RAnklePitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RAnkleRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RShoulderPitch", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RShoulderRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RElbowYaw", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)},
    {"RElbowRoll", gazebo::common::PID(480, 48, 4.8, 1, -1, 1000, -1000)}
  };
};

/// \brief The Nao type Zero from Simspark
class NaoSimsparkBT : public NaoBT
{
  // Documentation inherited
  public: virtual double Height() const
  {
    return 0.6;
  }

  // Documentation inherited
  public: virtual double TorsoHeight() const
  {
    return 0.39;
  }

  // Documentation inherited
  public: virtual const std::map<std::string, std::string>& BodyPartMap() const
  {
    return this->kBodyPartMap;
  }

  // Documentation inherited
  public: virtual
  const std::map<std::string, std::string>& HingeJointEffectorMap() const
  {
    return this->hingeJointEffectorMap;
  }

  // Documentation inherited
  public: virtual const std::map<std::string, std::string>&
  HingeJointPerceptorMap() const
  {
    return this->hingeJointPerceptorMap;
  }

  // Documentation inherited
  public: virtual std::map<std::string, gazebo::common::PID>&
  HingeJointPIDMap()
  {
    return this->hingeJointPIDMap;
  }

  // Documentation inherited
  public: virtual std::string CameraLinkName() const
  {
    return "Head";
  }

  // Documentation inherited
  public: virtual std::string TorsoLinkName() const
  {
    return "base_link";
  }

  // Documentation inherited
  public: virtual std::string LeftFootLinkName() const
  {
    return "l_sole";
  }

  // Documentation inherited
  public: virtual std::string RightFootLinkName() const
  {
    return "r_sole";
  }

  // Documentation inherited
  public: virtual std::string DefaultModelName() const
  {
    return "naoType0";
  }

  // Documentation inherited
  public: virtual std::string BlueModelPath() const
  {
    return "model://nao_type_zero";
  }

  // Documentation inherited
  public: virtual std::string RedModelPath() const
  {
    return "model://nao_type_zero";
  }

  // Documentation inherited
  protected: const std::map<std::string, std::string> kBodyPartMap =
  {
    {"head", "Head"},
    {"llowerarm", "LForeArm"},
    {"rlowerarm", "RForeArm"},
    {"lfoot", "l_sole"},
    {"rfoot", "r_sole"}
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
    {"raj3", "RElbowYaw"},
    {"raj4", "RElbowRoll"}
  };

  // documentation inherited
  protected: std::map<std::string, gazebo::common::PID>
  hingeJointPIDMap =
  {
      {"HeadYaw", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"HeadPitch", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LShoulderPitch", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LShoulderRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LElbowYaw", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LElbowRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LHipYawPitch", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LHipRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"LHipPitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"LKneePitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"LAnklePitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"LAnkleRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RHipYawPitch", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RHipRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RHipPitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"RKneePitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"RAnklePitch", gazebo::common::PID(400, 400, 0, 1, -1, 1000, -1000)},
      {"RAnkleRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RShoulderPitch", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RShoulderRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RElbowYaw", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)},
      {"RElbowRoll", gazebo::common::PID(160, 160, 0, 1, -1, 1000, -1000)}
  };
};

#endif
