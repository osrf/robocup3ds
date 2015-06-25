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

#include "robocup3ds/Geometry.hh"

using namespace ignition;

#define G_SQUARE(a) ((a) * (a))
#define DBL_EPSILON 2.2204460492503131e-16

/////////////////////////////////////////////////
bool Geometry::IntersectionCircunferenceLine(
  const math::Vector3<double> &_v, const math::Vector3<double> &p_c, double r,
  math::Vector3<double> &int1, math::Vector3<double> &int2)
{
  // Solve equations:
  // (x-px)^2 + (y - py)^2 = r^2
  // Ax + By + C = 0

  double i, j, k, A_2;
  double a, b, c;
  double tmp;
  math::Vector3<double> v;

  v = _v;

  if (fabs(v.X() - 0.0) < DBL_EPSILON)
  {
    // Avoid div by 0
    v.X() = DBL_EPSILON;
  }

  i = -2 * p_c.X();
  j = -2 * p_c.Y();
  k = G_SQUARE(p_c.X()) + G_SQUARE(p_c.Y()) - G_SQUARE(r);

  A_2 = G_SQUARE(v.X());
  a = G_SQUARE(-v.Y()) / A_2 + 1;
  b = -2 * v.Z() * -v.Y() / A_2  - v.Y() * i / v.X() + j;
  c = G_SQUARE(v.Z()) / A_2 - v.Z() * i / v.X() + k;

  // Solve a*y^2 + b+y + c = 0
  tmp = G_SQUARE(b) - 4 * a * c;
  if (tmp < 0)
  {
    // No intersection
    return false;
  }

  tmp = sqrt(tmp);
  int1.Y() = (-b + tmp) / (2 * a);
  int2.Y() = (-b - tmp) / (2 * a);

  int1.X() = (-v.Y() * int1.Y() - v.Z()) / v.X();
  int2.X() = (-v.Y() * int2.Y() - v.Z()) / v.X();

  return true;
}

/////////////////////////////////////////////////
bool Geometry::PointAbovePlane(const math::Vector3<double> &_pt,
                const math::Plane<double> &_plane)
{
  return (_plane.Normal().Dot(_pt) + _plane.Offset()) > DBL_EPSILON;
}

/////////////////////////////////////////////////
bool Geometry::IntersectionPlaneLine(const Line &_line,
                                     const math::Plane<double> &_plane,
                                     double &_t,
                                     math::Vector3<double> &_pt)
{
  math::Vector3<double> origin = _line.pts.at(0);
  math::Vector3<double> dir = _line.Dir();
  math::Vector3<double> normal = _plane.Normal();
  double D = _plane.Offset();

  double dp = normal.Dot(dir);
  if (fabs(dp) < DBL_EPSILON)
  {
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
bool Geometry::ClipPlaneLine(Line &_line, const math::Plane<double> &_plane)
{
  double t;
  math::Vector3<double> pt;

  bool isPt1AbovePlane = PointAbovePlane(_line.pts.at(0), _plane);
  bool isPt2AbovePlane = PointAbovePlane(_line.pts.at(1), _plane);

  if (isPt1AbovePlane && isPt2AbovePlane)
  {
    // do nothing
  }
  else if (!isPt1AbovePlane && isPt2AbovePlane)
  {
    if (IntersectionPlaneLine(_line, _plane, t, pt))
    { _line.Set(pt, _line.pts.at(0)); }
    else
    { return false; }
  }
  else if (isPt1AbovePlane && !isPt2AbovePlane)
  {
    if (IntersectionPlaneLine(_line, _plane, t, pt))
    { _line.Set(pt, _line.pts.at(1)); }
    else
    { return false; }
  }
  else if (!isPt1AbovePlane && !isPt2AbovePlane)
  {
    return false;
  }

  return true;
}
