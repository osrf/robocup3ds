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

#ifndef _GAZEBO_SOCCER_FIELD_PLUGIN_HH_
#define _GAZEBO_SOCCER_FIELD_PLUGIN_HH_

#include <ignition/math.hh>
#include <map>
#include <string>
#include <vector>

#include "Geometry.hh"

namespace SoccerField
{
  // Field dimensions.
  static const double CenterCircleRadius = 2;
  static const double FieldWidth = 30.0;
  static const double HalfFieldWidth = FieldWidth * 0.5;
  static const double FieldHeight = 20.0;
  static const double HalfFieldHeight = FieldHeight * 0.5;
  static const double GoalWidth = 2.1;
  static const double HalfGoalWidth = GoalWidth * 0.5;
  static const double GoalDepth = 0.6;
  static const double GoalHeight = 0.8;
  static const double RobotPoseHeight = 0.35;
  static const double BallRadius = 0.042;
  static const double OutofBoundsTol = BallRadius;
  static const double PenaltyBoxWidth = 3.9;
  static const double HalfPenaltyBoxWidth = PenaltyBoxWidth * 0.5;
  static const double PenaltyBoxDepth = 1.8;

  static const ignition::math::Vector3<double>
  GoalCenterLeft(-HalfFieldWidth, 0, 0);

  static const ignition::math::Vector3<double>
  GoalCenterRight(HalfFieldWidth, 0, 0);

  static const ignition::math::Vector3<double>
  BallCenterPosition(0, 0, BallRadius);

  static const ignition::math::Vector3<double>
  CenterOfField(0, 0, 0);

  static const ignition::math::Box GoalBoxLeft(
    ignition::math::Vector3<double>(-(GoalDepth + HalfFieldWidth),
                                    -HalfGoalWidth, -BallRadius),
    ignition::math::Vector3<double>
    (-HalfFieldWidth, HalfGoalWidth, GoalHeight));

  static const ignition::math::Box GoalBoxRight(
    ignition::math::Vector3<double>(GoalDepth + HalfFieldWidth,
                                    -HalfGoalWidth, -BallRadius),
    ignition::math::Vector3<double>
    (HalfFieldWidth, HalfGoalWidth, GoalHeight));

  static const ignition::math::Box PenaltyBoxLeft(
    ignition::math::Vector3<double>
    (-HalfFieldWidth, -HalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>
    (-HalfFieldWidth + PenaltyBoxDepth, HalfPenaltyBoxWidth, 10));

  static const ignition::math::Box PenaltyBoxRight(
    ignition::math::Vector3<double>
    (HalfFieldWidth, -HalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>
    (HalfFieldWidth - PenaltyBoxDepth, HalfPenaltyBoxWidth, 10));

  static const ignition::math::Box FieldLeft(ignition::math::Vector3<double>
      (-SoccerField::HalfFieldHeight, -SoccerField::HalfFieldWidth, 0),
      ignition::math::Vector3<double>(0, SoccerField::HalfFieldWidth, 0));

  static const ignition::math::Box FieldRight(ignition::math::Vector3<double>
      (0, -SoccerField::HalfFieldWidth, 0), ignition::math::Vector3<double>
      (SoccerField::HalfFieldHeight, SoccerField::HalfFieldWidth, 0));

  static const std::vector<ignition::math::Line3<double> > FieldLines =
  {
    // middle line
    ignition::math::Line3<double>(0, -10, 0, 10),
    // ground lines
    ignition::math::Line3<double>(15, -10, 15, 10),
    ignition::math::Line3<double>(-15, -10, -15, 10),
    // side lines
    ignition::math::Line3<double>(15, 10, -15, 10),
    ignition::math::Line3<double>(15, -10, -15, -10),
    // penalty lines
    ignition::math::Line3<double>(13.2, 3, 13.2, -3),
    ignition::math::Line3<double>(13.2, 3, 15, 3),
    ignition::math::Line3<double>(13.2, -3, 15, -3),
    ignition::math::Line3<double>(-13.2, 3, -13.2, -3),
    ignition::math::Line3<double>(-13.2, 3, -15, 3),
    ignition::math::Line3<double>(-13.2, -3, -15, -3),
    // center circle ring
    ignition::math::Line3<double>(2, 0, 1.618033989, 1.175570505),
    ignition::math::Line3<double>(1.618033989, 1.175570505,
    0.618033989, 1.902113033),
    ignition::math::Line3<double>(0.618033989, 1.902113033,
    -0.618033989, 1.902113033),
    ignition::math::Line3<double>(-0.618033989, 1.902113033,
    -1.618033989, 1.175570505),
    ignition::math::Line3<double>(-1.618033989, 1.175570505, -2, 0),
    ignition::math::Line3<double>(-2, 0, -1.618033989, -1.175570505),
    ignition::math::Line3<double>(-1.618033989, -1.175570505,
    -0.618033989, -1.902113033),
    ignition::math::Line3<double>(-0.618033989, -1.902113033,
    0.618033989, -1.902113033),
    ignition::math::Line3<double>(0.618033989, -1.902113033,
    1.618033989, -1.175570505),
    ignition::math::Line3<double>(1.618033989, -1.175570505, 2, 0)
  };

  static const std::map<std::string, ignition::math::Vector3<double>>
  LandMarks =
  {
    {"F1L", ignition::math::Vector3<double>(-15, 10, 0)},
    {"F1R", ignition::math::Vector3<double>(15, 10, 0)},
    {"F2R", ignition::math::Vector3<double>(15, -10, 0)},
    {"F2L", ignition::math::Vector3<double>(-15, -10, 0)},
    {"G1L", ignition::math::Vector3<double>(-15, 1.05, 0.8)},
    {"G1R", ignition::math::Vector3<double>(15, 1.05, 0.8)},
    {"G2L", ignition::math::Vector3<double>(-15, -1.05, 0.8)},
    {"G2R", ignition::math::Vector3<double>(15, -1.05, 0.8)}
  };
}

#endif
