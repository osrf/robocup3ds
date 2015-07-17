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

#include <netinet/in.h>
#include <algorithm>
#include <cmath>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/SoccerField.hh"

using namespace ignition;

bool Perceptor::useNoise = true;
const double Perceptor::kHearDist = 50.0;
const int Perceptor::bufferSize = 8000;

math::Vector3<double> Perceptor::fixedNoise(
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005));

math::Vector3<double> Perceptor::dNoiseSigma(0.0965, 0.1225, 0.1480);

/////////////////////////////////////////////////
Perceptor::Perceptor(GameState *const _gameState, RCPServer *const _server):
  gameState(_gameState),
  server(_server)
{
  this->buffer = std::make_shared<char>(Perceptor::bufferSize);
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
void Perceptor::UpdateLine(Agent &_agent,
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
void Perceptor::UpdateLandmark(Agent &_agent,
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
void Perceptor::UpdateOtherAgent(Agent &_agent,
                                 const Agent &_otherAgent) const
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

    AgentId otherAgentId(_otherAgent.uNum,
                         _otherAgent.team->name);
    _agent.percept.otherAgentBodyMap[otherAgentId][kv.first] =
      addNoise(Geometry::CartToPolar(_otherAgentPart));
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateAgentHear(Agent &_agent) const
{
  const AgentSay &say = this->gameState->say;
  AgentHear &hear = _agent.percept.hear;

  hear.isValid = false;
  if (!say.isValid)
  {
    return;
  }
  math::Vector3<double> relPos = this->G2LMat.TransformAffine(say.pos);
  if (relPos.Length() > Perceptor::kHearDist)
  {
    return;
  }

  hear.isValid = true;
  hear.gameTime = gameState->GetElapsedGameTime();
  hear.yaw = atan2(relPos.Y(), relPos.X());
  AgentId agentId(_agent.uNum, _agent.team->name);
  hear.self = say.agentId == agentId;
  hear.msg = say.msg;
}

/////////////////////////////////////////////////
int Perceptor::Serialize(const Agent &_agent, char *_string,
                         const int _size) const
{
  if (_size < 4000)
  { return 0; }

  int cx = 0;

  // write out perception info
  cx += snprintf(_string + cx, _size - cx, "(See");

  // write out landmark info
  for (auto &kv : _agent.percept.landMarks)
  {
    cx += SerializePoint(kv.first.c_str(), kv.second, _string + cx, _size - cx);
  }

  // write out other agent body parts
  for (auto &kv : _agent.percept.otherAgentBodyMap)
  {
    // agentNum = kv.first.first;
    // agentTeam = kv.first.second;
    cx += snprintf(_string + cx, _size - cx, " (P (team %s) (id %d)",
                   kv.first.second.c_str(), kv.first.first);
    for (auto &kv2 : kv.second)
    {
      cx += SerializePoint(kv2.first.c_str(),
                           kv2.second, _string + cx, _size - cx);
    }
    cx += snprintf(_string + cx, _size - cx, ")");
  }

  // write out fieldlines
  for (auto &fieldLine : _agent.percept.fieldLines)
  {
    cx += snprintf(_string + cx, _size - cx,
                   " (L (pol %.2f %.2f %.2f) (pol %.2f %.2f %.2f))",
                   fieldLine[0].X(), fieldLine[0].Y(), fieldLine[0].Z(),
                   fieldLine[1].X(), fieldLine[1].Y(), fieldLine[1].Z());
  }

  // finish writing out perception info
  cx += snprintf(_string + cx, _size - cx, ")");

  // write body joint angle info
  for (auto &kv : _agent.percept.hingeJoints)
  {
    cx += snprintf(_string + cx, _size - cx, " (HJ (n %s) (ax %.2f))",
                   kv.first.c_str(), kv.second);
  }

  // write out gyro info
  cx += SerializePoint("GYR (n torso)", _agent.percept.gyroRate,
                       _string + cx, _size - cx);

  // write out acceleration info
  cx += SerializePoint("GYR (n ACC)", _agent.percept.accel,
                       _string + cx, _size - cx);

  // write force resistance information
  cx += snprintf(_string + cx, _size - cx,
                 " (FRP (n lf) (c %.2f %.2f %.2f) (f %.2f %.2f %.2f))",
                 _agent.percept.leftFootFR.first.X(),
                 _agent.percept.leftFootFR.first.Y(),
                 _agent.percept.leftFootFR.first.Z(),
                 _agent.percept.leftFootFR.second.X(),
                 _agent.percept.leftFootFR.second.Y(),
                 _agent.percept.leftFootFR.second.Z());

  cx += snprintf(_string + cx, _size - cx,
                 " (FRP (n rf) (c %.2f %.2f %.2f) (f %.2f %.2f %.2f))",
                 _agent.percept.rightFootFR.first.X(),
                 _agent.percept.rightFootFR.first.Y(),
                 _agent.percept.rightFootFR.first.Z(),
                 _agent.percept.rightFootFR.second.X(),
                 _agent.percept.rightFootFR.second.Y(),
                 _agent.percept.rightFootFR.second.Z());

  return cx;
}

int Perceptor::SerializePoint(const char *_label,
                              const math::Vector3<double> &_pt,
                              char *_string, const int _size) const
{
  return snprintf(_string, _size, " (%s (pol %.2f %.2f %.2f))",
                  _label, _pt.X(), _pt.Y(), _pt.Z());
}

void Perceptor::SendToServer() const
{
  for (auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      int cx = this->Serialize(agent, &(this->buffer.get())[4],
                               Perceptor::bufferSize - 4);
      unsigned int _cx = htonl(static_cast<unsigned int>(cx));
      this->buffer.get()[0] = _cx         & 0xff;
      this->buffer.get()[1] = (_cx >> 8)  & 0xff;
      this->buffer.get()[2] = (_cx >> 16) & 0xff;
      this->buffer.get()[3] = (_cx >> 24) & 0xff;
      this->server->Send(agent.socketID, this->buffer.get(), cx + 4);
    }
  }
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> Perceptor::addNoise(
  const ignition::math::Vector3<double> &_pt) const
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
void Perceptor::SetG2LMat(const Agent &_agent)
{
  this->G2LMat = math::Matrix4<double>(_agent.cameraRot);
  this->G2LMat.Translate(_agent.cameraPos);
  this->G2LMat = this->G2LMat.Inverse();
}
