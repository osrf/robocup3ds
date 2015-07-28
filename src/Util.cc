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

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <map>
#include <string>

#include "robocup3ds/Util.hh"

/////////////////////////////////////////////////
gazebo::math::Vector3 I2G(const ignition::math::Vector3<double> _pt)
{
  return gazebo::math::Vector3(_pt.X(), _pt.Y(), _pt.Z());
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> G2I(const gazebo::math::Vector3 _pt)
{
  return ignition::math::Vector3<double>(_pt[0], _pt[1], _pt[2]);
}

/////////////////////////////////////////////////
gazebo::math::Quaternion I2G(const ignition::math::Quaternion<double> _q)
{
  return gazebo::math::Quaternion(_q.W(), _q.X(), _q.Y(), _q.Z());
}

/////////////////////////////////////////////////
ignition::math::Quaternion<double> G2I(const gazebo::math::Quaternion _q)
{
  auto qEuler = _q.GetAsEuler();
  return ignition::math::Quaternion<double>(qEuler[0], qEuler[1], qEuler[2]);
}

/////////////////////////////////////////////////
gazebo::math::Pose I2G(const ignition::math::Pose3<double> _p)
{
  return gazebo::math::Pose(I2G(_p.Pos()), I2G(_p.Rot()));
}

/////////////////////////////////////////////////
ignition::math::Pose3<double> G2I(const gazebo::math::Pose _p)
{
  return ignition::math::Pose3<double>(G2I(_p.pos), G2I(_p.rot));
}

/////////////////////////////////////////////////
bool LoadConfigParameter(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  double &_value)
{
  try
  {
    _value = std::stod(_config.at(_key));
  }
  catch (const std::exception &exc)
  {
    // gzerr << exc.what() << std::endl;
    return false;
  }
  gzmsg << "loaded param " << _key << " : " << _value << std::endl;
  return true;
}

/////////////////////////////////////////////////
bool LoadConfigParameterBool(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  bool &_boolValue)
{
  try
  {
    if (_config.at(_key) == "false" || _config.at(_key) == "0")
    {
      _boolValue = false;
    }
    else if (_config.at(_key) == "true" || _config.at(_key) == "1")
    {
      _boolValue = true;
    }
    else
    {
      return false;
    }
  }
  catch (const std::exception &exc)
  {
    // gzerr << exc.what() << std::endl;
    return false;
  }
  gzmsg << "KEY: " << _key << "\tVALUE: " << _boolValue << std::endl;
  return true;
}