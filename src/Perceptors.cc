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
#include <algorithm>
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

math::Vector3<double> Perceptor::fixedNoise(
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005));

math::Vector3<double> Perceptor::dNoiseSigma(0.0965, 0.1225, 0.1480);

/////////////////////////////////////////////////
Perceptor::Perceptor(GameState *_gameState):
  gameState(_gameState)
{
  double HFov = RAD(std::min(180.0, std::max(0.0, GameState::HFov)));
  double VFov = RAD(std::min(180.0, std::max(0.0, GameState::VFov)));

  math::Vector3<double> origin;
  math::Vector3<double> leftOrigin(0.0, 1.0, 0.0);
  math::Vector3<double> aboveOrigin(0.0, 1.0, 1.0);
  math::Vector3<double> upperRight(1.0, -tan(HFov / 2),
                                   tan(VFov / 2));
  math::Vector3<double> upperLeft(1.0, tan(HFov / 2),
                                  tan(VFov / 2));
  math::Vector3<double> lowerRight(1.0, -tan(HFov / 2),
                                   -tan(VFov / 2));
  math::Vector3<double> lowerLeft(1.0, tan(HFov / 2),
                                  -tan(VFov / 2));
  if (GameState::restrictVision)
  {
    this->viewFrustrum.push_back(
      math::Plane<double>(CALC_NORMAL(origin, aboveOrigin, leftOrigin)));
    this->viewFrustrum.push_back(
      math::Plane<double>(CALC_NORMAL(origin, lowerRight, upperRight)));
    this->viewFrustrum.push_back(
      math::Plane<double>(CALC_NORMAL(origin, upperRight, upperLeft)));
    this->viewFrustrum.push_back(
      math::Plane<double>(CALC_NORMAL(origin, upperLeft, lowerLeft)));
    this->viewFrustrum.push_back(
      math::Plane<double>(CALC_NORMAL(origin, lowerLeft, lowerRight)));
  }
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
                           const math::Line3<double> &_line) const
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
    agentLine.Set(addNoise(Geometry::CartToPolar(agentLine[0])),
                  addNoise(Geometry::CartToPolar(agentLine[1])));
    _agent.percept.fieldLines.push_back(agentLine);
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateLandmark(GameState::Agent &_agent,
                               const std::string &_landmarkname,
                               const math::Vector3<double>
                               &_landmark) const
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
    _agent.percept.landMarks[_landmarkname] =
      addNoise(Geometry::CartToPolar(_agentLandMark));
  }
}

ignition::math::Vector3<double>
Perceptor::addNoise(const ignition::math::Vector3<double> &_pt) const
{
  math::Vector3<double> newPt(
    _pt.X() + Perceptor::fixedNoise.X() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.X()) * _pt.X() * 0.01,
    _pt.Y() + Perceptor::fixedNoise.Y() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.Y()),
    _pt.Z() + Perceptor::fixedNoise.Z() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.Z()));

  // newPt.Set(round(newPt.X() * 100.0) / 100.0,
  //           round(newPt.Y() * 100.0) / 100.0,
  //           round(newPt.Z() * 100.0) / 100.0));

  return newPt;
}
