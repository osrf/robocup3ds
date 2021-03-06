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

#ifndef _GAZEBO_ROBOCUP3DS_UTIL_HH_
#define _GAZEBO_ROBOCUP3DS_UTIL_HH_

#include <gazebo/gazebo.hh>
#include <ignition/math.hh>
#include <map>
#include <string>

namespace Util
{
  /// \brief Helper function for loading configuration variables
  /// \param[in] _config Map of configuration variables
  /// \param[in] _key Key to look for in map
  /// \param[out] _value Value to return
  /// \return True if loading of parameter is successful
  bool LoadConfigParameter(
    const std::map<std::string, std::string> &_config,
    const std::string &_key,
    double &_value);

  /// \brief Helper function for loading configuration variables
  /// \param[in] _config Map of configuration variables
  /// \param[in] _key Key to look for in map
  /// \param[out] _boolValue Value to return
  /// \return True if loading of parameter is successful
  bool LoadConfigParameterBool(
    const std::map<std::string, std::string> &_config,
    const std::string &_key,
    bool &_boolValue);

  /// \brief Converts a c-string to double
  /// \param[in] _str C-string
  /// \param[out] _v Double value
  /// \return True if conversion is successful
  bool S2D(const char *_str, double &_v);
}

#endif
