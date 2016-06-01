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

#include <cerrno>
#include <cstdlib>
#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <map>
#include <string>

#include "robocup3ds/Util.hh"

/////////////////////////////////////////////////
bool Util::LoadConfigParameter(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  double &_value)
{
  if (_config.find(_key) == _config.end())
    return false;

  bool rValue = true;
  try
  {
    size_t offset;
    _value = std::stod(_config.at(_key), &offset);

    if (offset != _config.at(_key).size())
    {
      rValue = false;
    }
  }
  catch (const std::exception &exc)
  {
    rValue = false;
  }
  if (rValue)
  {
    gzmsg << "KEY: " << _key << " VALUE: " << _value << std::endl;
  }
  else
  {
    gzerr << "LoadConfigParameter() failed to read the following key: "
          << _key << ", using default values!" << std::endl;
  }
  return rValue;
}

/////////////////////////////////////////////////
bool Util::LoadConfigParameterBool(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  bool &_boolValue)
{
  if (_config.find(_key) == _config.end())
    return false;

  bool rValue = true;
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
      rValue = false;
    }
  }
  catch (const std::exception &exc)
  {
    rValue = false;
  }
  if (rValue)
  {
    gzmsg << "KEY: " << _key << " VALUE: " << _boolValue << std::endl;
  }
  else
  {
    gzerr << "LoadConfigParameterBool() failed to read the following key: "
          << _key << ", using default values!" << std::endl;
  }
  return rValue;
}

//////////////////////////////////////////////////
bool Util::S2D(const char *_str, double &_v)
{
  char *e;
  errno = 0;
  const double temp = std::strtod(_str, &e);

  // error, we didn't consume the entire string or overflow or underflow
  if (*e != '\0' || errno != 0 )
  {
    gzerr << "S2D() failed to read the following string: "
          << _str << std::endl;
    return false;
  }
  else
  {
    _v = temp;
    return true;
  }
}
