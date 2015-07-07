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
#include <utility>
#include <vector>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/SoccerField.hh"

using namespace ignition;

bool Perceptor::useNoise = true;
double Perceptor::hearDist = 50.0;

math::Vector3<double> Perceptor::fixedNoise(
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005));

math::Vector3<double> Perceptor::dNoiseSigma(0.0965, 0.1225, 0.1480);

/////////////////////////////////////////////////
Perceptor::Perceptor(const std::shared_ptr<GameState> &_gameState):
  gameState(_gameState)
{
  this->SetViewFrustum();
}

/////////////////////////////////////////////////
Perceptor::~Perceptor() {}

/////////////////////////////////////////////////
void Perceptor::SetViewFrustum()
{
  double HFov = RAD(std::min(180.0, std::max(0.0, GameState::HFov)));
  double VFov = RAD(std::min(180.0, std::max(0.0, GameState::VFov)));

  math::Vector3<double> origin = math::Vector3<double>::Zero;
  math::Vector3<double> upperRight(1.0, -tan(HFov / 2),
                                   tan(VFov / 2));
  math::Vector3<double> upperLeft(1.0, tan(HFov / 2),
                                  tan(VFov / 2));
  math::Vector3<double> lowerRight(1.0, -tan(HFov / 2),
                                   -tan(VFov / 2));
  math::Vector3<double> lowerLeft(1.0, tan(HFov / 2),
                                  -tan(VFov / 2));

  this->viewFrustum.clear();
  // forward facing plane (normal pointing down x-axis)
  this->viewFrustum.push_back(
    math::Plane<double>(math::Vector3<double>(1, 0, 0)));
  // right plane (normal pointing left)
  this->viewFrustum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, lowerRight, upperRight)));
  // top plane (normal pointing down)
  this->viewFrustum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, upperRight, upperLeft)));
  // left plane (normal pointing right)
  this->viewFrustum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, upperLeft, lowerLeft)));
  // bottom plane (normal pointing up)
  this->viewFrustum.push_back(
    math::Plane<double>(CALC_NORMAL(origin, lowerLeft, lowerRight)));
}

/////////////////////////////////////////////////
std::vector <ignition::math::Plane<double> > &Perceptor::GetViewFrustum()
{
  return this->viewFrustum;
}

/////////////////////////////////////////////////
void Perceptor::Update()
{
  for (auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      this->SetG2LMat(agent);

      // update line info
      agent.percept.fieldLines.clear();
      for (auto &fieldLine : SoccerField::FieldLines)
      {
        this->UpdateLine(agent, fieldLine);
      }

      // update landmark info
      agent.percept.landMarks.clear();
      for (auto &kv : SoccerField::LandMarks)
      {
        this->UpdateLandmark(agent, kv.first, kv.second);
      }

      // update ball info
      this->UpdateLandmark(agent, "B", this->gameState->GetBall());

      // update position and messages of other agents
      agent.percept.otherAgentBodyMap.clear();
      for (auto &otherTeam : this->gameState->teams)
      {
        for (auto &otherAgent : otherTeam->members)
        {
          if (otherAgent.uNum != agent.uNum ||
              otherAgent.team->name != agent.team->name)
          { this->UpdateOtherAgent(agent, otherAgent); }
        }
      }

      // update agent hear
      this->UpdateAgentHear(agent);
    }
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateLine(GameState::Agent &_agent,
                           const math::Line3<double> &_line) const
{
  math::Line3<double> agentLine(
    this->G2LMat.TransformAffine(_line[0]),
    this->G2LMat.TransformAffine(_line[1]));

  if (GameState::restrictVision)
  {
    for (auto &viewPlane : this->viewFrustum)
    {
      // std::cout << i << " // " << agentLine << " // " <<
      //           this->viewFrustum.at(i).Normal() << " // " << validLine
      //           << std::endl;
      if (!Geometry::ClipPlaneLine(agentLine, viewPlane))
      { return; }
    }
  }

  agentLine.Set(addNoise(Geometry::CartToPolar(agentLine[0])),
                addNoise(Geometry::CartToPolar(agentLine[1])));
  _agent.percept.fieldLines.push_back(agentLine);
}

/////////////////////////////////////////////////
void Perceptor::UpdateLandmark(GameState::Agent &_agent,
                               const std::string &_landmarkname,
                               const math::Vector3<double>
                               &_landmark) const
{
  math::Vector3<double> _agentLandMark =
    this->G2LMat.TransformAffine(_landmark);

  if (GameState::restrictVision)
  {
    for (auto &viewPlane : this->viewFrustum)
    {
      if (!Geometry::PointAbovePlane(_agentLandMark, viewPlane))
      { return; }
    }
  }

  _agent.percept.landMarks[_landmarkname] =
    addNoise(Geometry::CartToPolar(_agentLandMark));
}

/////////////////////////////////////////////////
void Perceptor::UpdateOtherAgent(GameState::Agent &_agent,
                                 const GameState::Agent &_otherAgent) const
{
  for (auto &kv : _otherAgent.selfBodyMap)
  {
    math::Vector3<double> _otherAgentPart =
      this->G2LMat.TransformAffine(kv.second);

    if (GameState::restrictVision)
    {
      for (auto &viewPlane : this->viewFrustum)
      {
        if (!Geometry::PointAbovePlane(_otherAgentPart, viewPlane))
        { return; }
      }
    }

    GameState::AgentId otherAgentId(_otherAgent.uNum,
                                    _otherAgent.team->name);
    _agent.percept.otherAgentBodyMap[otherAgentId][kv.first] =
      addNoise(Geometry::CartToPolar(_otherAgentPart));
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateAgentHear(GameState::Agent &_agent) const
{
  const GameState::AgentSay &say = this->gameState->say;
  GameState::AgentHear &hear = _agent.percept.hear;

  hear.isValid = false;
  if (!say.isValid)
  {
    return;
  }
  math::Vector3<double> relPos = this->G2LMat.TransformAffine(say.pos);
  if (relPos.Length() > Perceptor::hearDist)
  {
    return;
  }

  hear.isValid = true;
  hear.gameTime = gameState->GetElapsedGameTime();
  hear.yaw = atan2(relPos.Y(), relPos.X());
  GameState::AgentId agentId(_agent.uNum, _agent.team->name);
  hear.self = say.agentId == agentId;
  hear.msg = say.msg;
}

/////////////////////////////////////////////////
ignition::math::Vector3<double>
Perceptor::addNoise(const ignition::math::Vector3<double> &_pt) const
{
  if (!Perceptor::useNoise)
  {
    return _pt;
  }

  math::Vector3<double> newPt(
    _pt.X() + Perceptor::fixedNoise.X() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.X()) * _pt.X() * 0.01,
    _pt.Y() + Perceptor::fixedNoise.Y() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.Y()),
    _pt.Z() + Perceptor::fixedNoise.Z() +
    math::Rand::DblNormal(0, Perceptor::dNoiseSigma.Z()));

  // truncation should be done when serializing the messages?
  // newPt.Set(round(newPt.X() * 100.0) / 100.0,
  //           round(newPt.Y() * 100.0) / 100.0,
  //           round(newPt.Z() * 100.0) / 100.0));

  return newPt;
}

/////////////////////////////////////////////////
void Perceptor::SetG2LMat(const GameState::Agent &_agent)
{
  this->G2LMat = math::Matrix4<double>(_agent.cameraRot);
  this->G2LMat.Translate(_agent.pos);
  this->G2LMat = this->G2LMat.Inverse();
}
