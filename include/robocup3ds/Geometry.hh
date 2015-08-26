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

/// \brief Calculates normal of plane given three points
#define CALC_NORMAL(P_A, P_B, P_C) ((P_C - P_A).Cross(P_B - P_A)).Normalize()

namespace Geometry
{
  /// \brief Calculates the intersection between a line and a plane
  /// \param[in] _line Line object
  /// \param[in] _plane Plane object
  /// \param[out] _t T value corresponding to intersection point.
  /// The intersection point can be derived from _t using the following formula:
  /// _line[0] + t * (_line[1] - _line[0])
  /// If _t < 0 or _t > 1, it means intersection point exceeds the endpoints of
  /// the line.
  /// \param[out] _pt Intersection point
  /// \return True when line intersects with plane
  bool IntersectionPlaneLine(const ignition::math::Line3<double> &_line,
                             const ignition::math::Plane<double> &_plane,
                             double &_t,
                             ignition::math::Vector3<double> &_pt);

  /// \brief Clips a line to the positive side of plane if necessary
  /// \param[out] _line Line object
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

  /// \brief Calculates the intersection between a 2D circle (XY Cartesian
  /// coordinates) and a line3D's 2D projection onto the XY plane.
  /// \param[in] _line 3D line object
  /// \param[in] _circleCenter Center of the circle
  /// \param[in] _r Radius of the circle.
  /// \param[out] _int1 Vector3 with the coordinates of the first intersection
  /// point.
  /// \param[out] _int2 Vector3 with the coordinates of the second intersection
  /// point.
  /// \return True if line intersects circle
  bool IntersectionCircumferenceLine(const ignition::math::Line3<double> &_line,
                                     const ignition::math::Vector3<double>
                                     &_circleCenter,
                                     double _r,
                                     ignition::math::Vector3<double> &_int1,
                                     ignition::math::Vector3<double> &_int2);


  /// \brief Transform from spherical to cartesian coordinates, spherical
  /// coordinates are in the following order: radius, inclination, azimuth
  /// \param[in] _pt Point in spherical coordinates
  /// \return Point in cartesian coordinates
  ignition::math::Vector3<double> SphereToCart(
    const ignition::math::Vector3<double> &_pt);

  /// \brief Transform from cartesian to spherical coordinates
  /// \param[in] _pt Point in cartesian coordinates
  /// \return Point in spherical coordinates
  ignition::math::Vector3<double> CartToSphere(
    const ignition::math::Vector3<double> &_pt);
}

#endif
