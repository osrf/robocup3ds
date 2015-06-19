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

#ifndef _GAZEBO_GEOMETRY_HH_
#define _GAZEBO_GEOMETRY_HH_

#include <ignition/math.hh>

namespace Geometry {

  /// \brief Calculates the intersection between a circumference and a line
  /// passing through its center.
  /// \param[in] v Vector director of the line.
  /// \param[in] p_c Center of the circunference and point of the line.
  /// \param[in] r Radius of the circunference.
  /// \param[out] int1 Vector3 with the coordinates of the first intersection
  /// point.
  /// \param[out] int2 Vector3 with the coordinates of the second intersection
  /// point.
  bool IntersectionCircunferenceLine(const ignition::math::Vector3<double> &v,
                                     const ignition::math::Vector3<double> &p_c,
                                     double r,
                                     ignition::math::Vector3<double> &int1,
                                     ignition::math::Vector3<double> &int2);
}

#endif
