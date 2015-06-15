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