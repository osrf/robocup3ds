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
 * WITHOUT WARRANTIES or CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions &&
 * limitations under the License.
 *
*/
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/SoccerField.hh"

#define RAD(X) X * 180.0 / M_PI
#define DEG(X) X * M_PI / 180.0
#define CALC_NORMAL(P_A, P_B, P_C) ((P_C - P_A).Cross(P_B - P_A)).Normalize()

using namespace ignition;

/////////////////////////////////////////////////
Perceptor::Perceptor(GameState *_gameState, double _HFov, double _VFov):
  gameState(_gameState),
  HFov(_HFov),
  VFov(_VFov)
{
  math::Vector3<double> origin;
  math::Vector3<double> upperRight(1.0, -tan(this->HFov / 2),
                                   tan(this->VFov / 2));
  math::Vector3<double> upperLeft(1.0, tan(this->HFov / 2),
                                  tan(this->VFov / 2));
  math::Vector3<double> lowerRight(1.0, -tan(this->HFov / 2),
                                   -tan(this->VFov / 2));
  math::Vector3<double> lowerLeft(1.0, tan(this->HFov / 2),
                                  -tan(this->VFov / 2));

  this->viewFrustrum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, lowerRight, upperRight)));
  this->viewFrustrum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, upperRight, upperLeft)));
  this->viewFrustrum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, upperLeft, lowerLeft)));
  this->viewFrustrum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, lowerLeft, lowerRight)));
}

/////////////////////////////////////////////////
void Perceptor::Update()
{
  for (size_t i = 0; i < this->gameState->teams.size(); ++i)
  {
    std::shared_ptr<GameState::Team> team = this->gameState->teams.at(i);
    for (size_t j = 0; j < team->members.size(); ++j)
    {
      GameState::Agent &agent = team->members.at(j);

      // update line info
      agent.percept.fieldLines.clear();
      for (size_t k = 0; k < SoccerField::FieldLines.size(); ++k)
      {
        this->UpdateLine(agent, SoccerField::FieldLines.at(k));
      }

      // update landmark info
      agent.percept.landMarks.clear();
      for (auto iter = SoccerField::LandMarks.begin();
           iter != SoccerField::LandMarks.end(); ++iter)
      {
        this->UpdateLandmark(agent, iter->first, iter->second);
      }

      // update ball info
      this->UpdateLandmark(agent, "B", this->gameState->GetBall());
    }
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateLine(GameState::Agent &_agent,
                           const math::Line3<double> &_line)
{
  math::Line3<double> agentLine(
    _agent.cameraRot.Inverse() * (_line[0] - _agent.pos),
    _agent.cameraRot.Inverse() * (_line[1] - _agent.pos));

  bool validLine = true;
  for (size_t i = 0; i < this->viewFrustrum.size(); ++i)
  {
    validLine = Geometry::ClipPlaneLine(agentLine, this->viewFrustrum.at(i));
    if (!validLine)
    { break; }
  }

  if (validLine)
  {
    _agent.percept.fieldLines.push_back(agentLine);
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateLandmark(GameState::Agent &_agent,
                               const std::string &_landmarkname,
                               const math::Vector3<double>
                               &_landmark)
{
  math::Vector3<double> _agentLandMark = _agent.cameraRot.Inverse() *
                                         (_landmark - _agent.pos);

  bool validLandMark = true;
  for (size_t i = 0; i < this->viewFrustrum.size(); ++i)
  {
    validLandMark = Geometry::PointAbovePlane(_agentLandMark,
                    this->viewFrustrum.at(i));
    if (!validLandMark)
    { break; }
  }

  if (validLandMark)
  {
    _agent.percept.landMarks[_landmarkname] = _agentLandMark;
  }
}
