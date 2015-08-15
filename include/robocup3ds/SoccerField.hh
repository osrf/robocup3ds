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

#ifndef _GAZEBO_ROBOCUP3DS_SOCCERFIELD_HH_
#define _GAZEBO_ROBOCUP3DS_SOCCERFIELD_HH_

#include <ignition/math.hh>
#include <map>
#include <string>
#include <vector>

/// \brief Field dimensions and locations/sizes of goals and lines. All
/// measurements are in meters
namespace SoccerField
{

  /// \brief Size of center circle
  static const double kCenterCircleRadius = 2;

  /// \brief Width of field (X-axis)
  static const double kFieldWidth = 30.0;

  /// \brief Width of half a field
  static const double kHalfFieldWidth = kFieldWidth * 0.5;

  /// \brief Height of field (Y-axis)
  static const double kFieldHeight = 20.0;

  /// \brief Height of half a field
  static const double kHalfFieldHeight = kFieldHeight * 0.5;

  /// \brief Width of goal
  static const double kGoalWidth = 2.1;

  /// \brief Width of half a goal
  static const double kHalfGoalWidth = kGoalWidth * 0.5;

  /// \brief Depth of goal
  static const double kGoalDepth = 0.6;

  /// \brief Height of goal
  static const double kGoalHeight = 0.8;

  /// \brief Radius of ball
  static const double kBallRadius = 0.04;

  /// \brief How much ball to have travel beyond field lines to count as out
  /// of bounds
  static const double kOutofBoundsTol = kBallRadius;

  /// \brief Width of penalty box
  static const double kPenaltyBoxWidth = 3.9;

  /// \brief Width of half a penalty box
  static const double kHalfPenaltyBoxWidth = kPenaltyBoxWidth * 0.5;

  /// \brief Depth of penalty box
  static const double kPenaltyBoxDepth = 1.8;

  /// \brief Center of left goal
  static const ignition::math::Vector3<double>kGoalCenterLeft(
    -kHalfFieldWidth, 0, 0);

  /// \brief Center of right goal
  static const ignition::math::Vector3<double>kGoalCenterRight(
    kHalfFieldWidth, 0, 0);

  /// \brief Position of ball in center of the field
  static const ignition::math::Vector3<double>kBallCenterPosition(
    0, 0, kBallRadius);

  /// \brief Center of the field
  static const ignition::math::Vector3<double>kCenterOfField(
    0, 0, 0);

  /// \brief Left goal plane, if ball travels through the plane,
  /// it counts as a goal
  static const ignition::math::Plane<double> kGoalPlaneLeft(
    ignition::math::Vector3<double>(1, 0, 0), kHalfFieldWidth);

  /// \brief Left goal plane, if ball travels through the plane,
  /// it counts as a goal
  static const ignition::math::Plane<double> GoalPlaneRight(
    ignition::math::Vector3<double>(1, 0, 0), -kHalfFieldWidth);

  /// \brief Left goal box, if ball is inside box
  /// it counts as a goal
  static const ignition::math::Box kGoalBoxLeft(
    ignition::math::Vector3<double>(-(kGoalDepth + kHalfFieldWidth),
                                    -kHalfGoalWidth, -kBallRadius),
    ignition::math::Vector3<double>
    (-kHalfFieldWidth, kHalfGoalWidth, kGoalHeight));

  /// \brief Right goal box, if ball is inside box
  /// it counts as a goal
  static const ignition::math::Box kGoalBoxRight(
    ignition::math::Vector3<double>(kGoalDepth + kHalfFieldWidth,
                                    -kHalfGoalWidth, -kBallRadius),
    ignition::math::Vector3<double>
    (kHalfFieldWidth, kHalfGoalWidth, kGoalHeight));

  /// \brief Left penalty box
  static const ignition::math::Box kPenaltyBoxLeft(
    ignition::math::Vector3<double>
    (-kHalfFieldWidth, -kHalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>
    (-kHalfFieldWidth + kPenaltyBoxDepth, kHalfPenaltyBoxWidth, 10));

  /// \brief Right penalty box
  static const ignition::math::Box kPenaltyBoxRight(
    ignition::math::Vector3<double>
    (kHalfFieldWidth, -kHalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>
    (kHalfFieldWidth - kPenaltyBoxDepth, kHalfPenaltyBoxWidth, 10));

  /// \brief Bounding box around left side of field
  static const ignition::math::Box FieldLeft(ignition::math::Vector3<double>
      (-kHalfFieldHeight, -kHalfFieldWidth, -10),
      ignition::math::Vector3<double>(0, kHalfFieldWidth, 10));

  /// \brief Bounding box around right side of field
  static const ignition::math::Box FieldRight(ignition::math::Vector3<double>
      (0, -kHalfFieldWidth, -10), ignition::math::Vector3<double>
      (kHalfFieldHeight, kHalfFieldWidth, 10));

  /// \brief Vector of all the field lines on field
  static const std::vector<ignition::math::Line3<double> > kFieldLines =
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

  /// \brief Vector of all the landmarks on field
  static const std::map<std::string, ignition::math::Vector3<double>>
      kLandMarks =
  {
    {
      "F1L", ignition::math::Vector3<double>(
        -kHalfFieldWidth, kHalfFieldHeight, 0)
    },
    {
      "F1R", ignition::math::Vector3<double>(
        kHalfFieldWidth, kHalfFieldHeight, 0)
    },
    {
      "F2R", ignition::math::Vector3<double>(
        kHalfFieldWidth, -kHalfFieldHeight, 0)
    },
    {
      "F2L", ignition::math::Vector3<double>(
        -kHalfFieldWidth, -kHalfFieldHeight, 0)
    },
    {
      "G1L", ignition::math::Vector3<double>(
        -kHalfFieldWidth, kHalfGoalWidth, kGoalHeight)
    },
    {
      "G1R", ignition::math::Vector3<double>(
        kHalfFieldWidth, kHalfGoalWidth, kGoalHeight)
    },
    {
      "G2L", ignition::math::Vector3<double>(
        -kHalfFieldWidth, -kHalfGoalWidth, kGoalHeight)
    },
    {
      "G2R", ignition::math::Vector3<double>(
        kHalfFieldWidth, -kHalfGoalWidth, kGoalHeight)
    }
  };

  /// \brief Name of ball model
  static const std::string kBallName = "soccer_ball";

  /// \brief Name of link of ball model
  static const std::string kBallLinkName = "soccer_ball_link";
}

#endif
