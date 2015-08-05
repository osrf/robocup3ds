/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * You may not use this file except in compliance with the License.
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
#include <cfloat>
#include <cmath>
#include <iostream>

#include "robocup3ds/Geometry.hh"

using namespace ignition;

/////////////////////////////////////////////////
bool Geometry::IntersectionCircumferenceLine(
  const math::Line3<double> &_line, const math::Vector3<double> &_pc, double _r,
  math::Vector3<double> &_int1, math::Vector3<double> &_int2)
{
  math::Vector3<double> localP1 = _line[0] - _pc;
  math::Vector3<double> localP2 = _line[1] - _pc;
  math::Vector3<double> localDir = localP2 - localP1;

  double a = G_SQUARE(localDir.X()) + G_SQUARE(localDir.Y());
  double b = 2 * ((localDir.X() * localP1.X()) + (localDir.Y() * localP1.Y()));
  double c = G_SQUARE(localP1.X()) + G_SQUARE(localP1.Y()) - G_SQUARE(_r);

  double delta = G_SQUARE(b) - (4 * a * c);

  if (delta < DBL_EPSILON)
  {
    return false;
  }

  double sqrtDelta = sqrt(delta);
  double u1 = (-b - sqrtDelta) / (2 * a);
  double u2 = (-b + sqrtDelta) / (2 * a);

  _int1.Set(_line[0].X() + (u1 * localDir.X()),
            _line[0].Y() + (u1 * localDir.Y()),
            _int1.Z());

  _int2.Set(_line[0].X() + (u2 * localDir.X()),
            _line[0].Y() + (u2 * localDir.Y()),
            _int2.Z());

  return true;
}

/////////////////////////////////////////////////
bool Geometry::PointAbovePlane(const math::Vector3<double> &_pt,
                               const math::Plane<double> &_plane)
{
  return _plane.Side(_pt) == math::Plane<double>::POSITIVE_SIDE;
}

/////////////////////////////////////////////////
bool Geometry::IntersectionPlaneLine(
  const math::Line3<double> &_line,
  const math::Plane<double> &_plane,
  double &_t,
  math::Vector3<double> &_pt)
{
  math::Vector3<double> origin = _line[0];
  math::Vector3<double> dir = _line.Direction() * _line.Length();
  math::Vector3<double> normal = _plane.Normal();
  double D = _plane.Offset();

  double dp = normal.Dot(dir);
  if (fabs(dp) < DBL_EPSILON)
  {
    // line does not intersect plane
    return false;
  }
  //       (A*origin.x + B*origin.y + C*origin.z + D)
  // t = - ------------------------------------------
  //             (A*dir.x + B*dir.y + C*dir.z)
  _t = -(normal.Dot(origin) + D) / dp;
  _pt = origin + _t * dir;
  return true;
}

/////////////////////////////////////////////////
bool Geometry::ClipPlaneLine(math::Line3<double> &_line,
                             const math::Plane<double> &_plane)
{
  double t;
  math::Vector3<double> pt;

  bool isPt1AbovePlane = PointAbovePlane(_line[0], _plane);
  bool isPt2AbovePlane = PointAbovePlane(_line[1], _plane);

  if (isPt1AbovePlane && isPt2AbovePlane)
  {
    // do nothing
  }
  else if (!isPt1AbovePlane && isPt2AbovePlane)
  {
    if (IntersectionPlaneLine(_line, _plane, t, pt))
    { _line.SetA(pt); }
  }
  else if (isPt1AbovePlane && !isPt2AbovePlane)
  {
    if (IntersectionPlaneLine(_line, _plane, t, pt))
    { _line.SetB(pt); }
  }
  else if (!isPt1AbovePlane && !isPt2AbovePlane)
  {
    return false;
  }

  return true;
}

/////////////////////////////////////////////////
math::Vector3<double> Geometry::CartToSphere(const math::Vector3<double> &_pt)
{
  double r = _pt.Length() + DBL_EPSILON;
  return math::Vector3<double>(r, atan2(_pt.Y(), _pt.X()), acos(_pt.Z() / r));
}

/////////////////////////////////////////////////
math::Vector3<double> Geometry::SphereToCart(const math::Vector3<double> &_pt)
{
  return math::Vector3<double>(_pt.X() * sin(_pt.Z()) * cos(_pt.Y()),
                               _pt.X() * sin(_pt.Z()) * sin(_pt.Y()),
                               _pt.X() * cos(_pt.Z()));
}
