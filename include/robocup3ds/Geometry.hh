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

#ifndef _GAZEBO_ROBOCUP3DS_GEOMETRY_HH_
#define _GAZEBO_ROBOCUP3DS_GEOMETRY_HH_

#include <ignition/math.hh>
#include <vector>

/// \brief Squaring function
#define G_SQUARE(a) ((a) * (a))

/// \brief Converts radians to degrees
#define DEG(X) X * (180.0 / M_PI)

/// \brief Converts degrees to radians
#define RAD(X) X * (M_PI / 180.0)

/// \brief Calculates normal of plane given three points
#define CALC_NORMAL(P_A, P_B, P_C) ((P_C - P_A).Cross(P_B - P_A)).Normalize()

namespace Geometry
{
  /// \brief Calculates the intersection between a line and a plane
  /// \param[in] _line Line object
  /// \param[in] _plane Plane object
  /// \param[out] _t T value where intersection occurs
  /// \param[out] _pt Intersection point
  /// \return True when line intersects with plane
  bool IntersectionPlaneLine(const ignition::math::Line3<double> &_line,
                             const ignition::math::Plane<double> &_plane,
                             double &_t,
                             ignition::math::Vector3<double> &_pt);

  /// \brief Clips a line to a plane if necessary
  /// \param[in] _line Line object
  /// \param[in] _plane Plane object
  /// \return True when line still exists after clipping
  bool ClipPlaneLine(ignition::math::Line3<double> &_line,
                     const ignition::math::Plane<double> &_plane);

  /// \brief Whether a point is above plane (same side as normal) or below it
  /// \param[in] _pt Point object
  /// \param[in] _plane Plane object
  /// \return True if point is above plane
  bool PointAbovePlane(const ignition::math::Vector3<double> &_pt,
                       const ignition::math::Plane<double> &_plane);

  /// \brief Calculates the 2D intersection between a circumference and a line
  /// passing through its center.
  /// \param[in] _line 3D line object
  /// \param[in] _pc Center of the circumference and point of the line.
  /// \param[in] _r Radius of the circumference.
  /// \param[out] _int1 Vector3 with the coordinates of the first intersection
  /// point.
  /// \param[out] _int2 Vector3 with the coordinates of the second intersection
  /// point.
  bool IntersectionCircunferenceLine(const ignition::math::Line3<double> &_line,
                                     const ignition::math::Vector3<double> &_pc,
                                     double _r,
                                     ignition::math::Vector3<double> &_int1,
                                     ignition::math::Vector3<double> &_int2);


  /// \brief Transform from polar to cartesian coordinates
  /// \param[in] _pt Point object
  ignition::math::Vector3<double> PolarToCart(
    const ignition::math::Vector3<double> &_pt);

  /// \brief Transform from cartesian to polar coordinates
  /// \param[in] _pt Point object
  ignition::math::Vector3<double> CartToPolar(
    const ignition::math::Vector3<double> &_pt);
}

#endif
