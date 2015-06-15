/*
 * Copyright (C) 2014 Open Source Robotics Foundation
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

namespace SoccerField
{
  // Field dimensions.
  static const double CenterCircleRadius = 2;
  static const double FieldWidth = 30.0;
  static const double HalfFieldWidth = FieldWidth * 0.5;
  static const double FieldHeight = 20.0;
  static const double HalfFieldHeight = FieldHeight * 0.5;
  static const double FreeKickMoveDist = 2.0;
  static const double FreeKickDist = 9.15;
  static const double GoalWidth = 2.1;
  static const double HalfGoalWidth = GoalWidth * 0.5;
  static const double GoalDepth = 0.6;
  static const double GoalHeight = 0.8;
  static const double NaoPoseHeight = 0.35;
  static const double BallRadius = 0.042;
  static const double OutofBoundsTol = BallRadius;
  static const double PenaltyBoxWidth = 3.9;
  static const double HalfPenaltyBoxWidth = PenaltyBoxWidth * 0.5;
  static const double PenaltyBoxDepth = 1.8;
  static const ignition::math::Vector3<double> GoalCenterLeft(-HalfFieldWidth, 0, 0);
  static const ignition::math::Vector3<double> GoalCenterRight(HalfFieldWidth, 0, 0);
  static const ignition::math::Vector3<double> BallCenterPosition(0, 0, BallRadius);
  static const ignition::math::Vector3<double> CenterOfField(0, 0, 0);

  static const ignition::math::Box GoalBoxLeft(
    ignition::math::Vector3<double>(-(GoalDepth + HalfFieldWidth), -HalfGoalWidth, -BallRadius),
    ignition::math::Vector3<double>(-HalfFieldWidth, HalfGoalWidth, GoalHeight));

  static const ignition::math::Box GoalBoxRight(
    ignition::math::Vector3<double>(GoalDepth + HalfFieldWidth, -HalfGoalWidth, -BallRadius),
    ignition::math::Vector3<double>(HalfFieldWidth, HalfGoalWidth, GoalHeight));

  static const ignition::math::Box PenaltyBoxLeft(
    ignition::math::Vector3<double>(-HalfFieldWidth, -HalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>(-HalfFieldWidth + PenaltyBoxDepth, HalfPenaltyBoxWidth, 10)
  );

  static const ignition::math::Box PenaltyBoxRight(
    ignition::math::Vector3<double>(HalfFieldWidth, -HalfPenaltyBoxWidth, -10),
    ignition::math::Vector3<double>(HalfFieldWidth - PenaltyBoxDepth, HalfPenaltyBoxWidth, 10)
  );


  static const ignition::math::Box FieldLeft(ignition::math::Vector3<double>(-SoccerField::HalfFieldHeight, -SoccerField::HalfFieldWidth, 0), ignition::math::Vector3<double>(0, SoccerField::HalfFieldWidth, 0));
  static const ignition::math::Box FieldRight(ignition::math::Vector3<double>(0, -SoccerField::HalfFieldWidth, 0), ignition::math::Vector3<double>(SoccerField::HalfFieldHeight, SoccerField::HalfFieldWidth, 0));

  // static std::vector<ignition::math::Pose3<double> > leftKickOffPose{
  //   ignition::math::Pose3<double>(-0.2, -0.3, NaoPoseHeight, 0, 0, 0.5),
  //   ignition::math::Pose3<double>(-0.2, 0.3, NaoPoseHeight, 0, 0, -0.5),
  //   ignition::math::Pose3<double>(-2.0, -0.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.0, 2.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.0, -2.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.0, 0.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-10.0, 3.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-10, 1.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-10.0, -1.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-10.0, -3.5, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-SoccerField::HalfFieldHeight + 0.5, 0, NaoPoseHeight, 0, 0, 0)};

  // static std::vector<ignition::math::Pose3<double> > rightKickOffPose{
  //   ignition::math::Pose3<double>(0.5, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(2.0, -3.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(2.0, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(2.0, 3.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(4.0, -4.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(4.0, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(4.0, 4.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(6.0, -5.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(6.0, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(6.0, 5.5, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(SoccerField::HalfFieldHeight - 0.5, 0, NaoPoseHeight, 0, 0, 3.14)};

  // static std::vector<ignition::math::Pose3<double> > leftInitPose{
  //   ignition::math::Pose3<double>(-2.5, 0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-3.5, -2.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-3.5, 0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-3.5, 2.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.5, -4.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.5, 0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-5.5, 4.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-7.5, -5.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-7.5, 0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-7.5, 5.0, NaoPoseHeight, 0, 0, 0),
  //   ignition::math::Pose3<double>(-SoccerField::HalfFieldHeight + 0.5, 0, NaoPoseHeight, 0, 0, 0)};

  // static std::vector<ignition::math::Pose3<double> > rightInitPose{
  //   ignition::math::Pose3<double>(2.5, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(3.5, -2.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(3.5, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(3.5, 2.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(5.5, -4.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(5.5, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(5.5, 4.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(7.5, -5.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(7.5, 0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(7.5, 5.0, NaoPoseHeight, 0, 0, 3.14),
  //   ignition::math::Pose3<double>(SoccerField::HalfFieldHeight - 0.5, 0, NaoPoseHeight, 0, 0, 3.14)};
}

#endif
