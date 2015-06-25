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
#include <vector>

namespace Geometry
{
  /// \class Line Geometry.hh robocup3ds/Geometry.hh
  /// \brief 3D Line object
  class Line
  {
    /// \brief Constructor with two default points
    public: Line()
    {
      this->pts.push_back(ignition::math::Vector3<double>());
      this->pts.push_back(ignition::math::Vector3<double>());
    }

    /// \brief Constructor of line from two points
    public: Line(ignition::math::Vector3<double> _a,
         ignition::math::Vector3<double> _b)
    {
      this->pts.push_back(_a);
      this->pts.push_back(_b);
    }

    /// \brief Constructor of a line from only xy coordinates
    public: Line(double _x1, double _y1, double _x2, double _y2)
    {
      this->pts.push_back(ignition::math::Vector3<double>(_x1, _y1, 0));
      this->pts.push_back(ignition::math::Vector3<double>(_x2, _y2, 0));
    }

    /// \brief Get direction vector, used to get parameterized line equation
    /// \return Direction vector
    public: ignition::math::Vector3<double> Dir() const
    {
      return this->pts.at(1) - this->pts.at(0);
    }

    /// \brief Get the length of line
    /// \return Length of line
    public: double Length() const
    {
      return this->Dir().Length();
    }

    /// \brief Set the line
    public: void Set(const ignition::math::Vector3<double> _a,
         const ignition::math::Vector3<double> _b)
    {
      this->pts.at(0) = _a;
      this->pts.at(1) = _b;
    }

    /// \brief Starting and ending points of line
    public: std::vector<ignition::math::Vector3<double> > pts;
  };

  /// \brief Calculates the intersection between a line and a plane
  /// \param[in] Line object
  /// \param[in] Plane object
  /// \param[out] T value where intersection occurs
  /// \param[out] Intersection point
  /// \return Whether line is not parallel with plane
  bool IntersectionPlaneLine(const Line &_line,
    const ignition::math::Plane<double> &_plane, double &_t,
    ignition::math::Vector3<double> & _pt);

  /// \brief Clips a line to a plane if necessary
  /// \param[in] Line object
  /// \param[in] Plane object
  /// \return Whether line is still valid after clipping
  bool ClipPlaneLine(Line &_line, const ignition::math::Plane<double> &_plane);

  /// \brief Whether a point is above plane or below it
  /// \param[in] Point object
  /// \param[in] Plane object
  /// \return True if point is above plane
  bool PointAbovePlane(const ignition::math::Vector3<double> &_pt,
    const ignition::math::Plane<double> &_plane);

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
