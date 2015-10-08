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
#include <string>
#include <vector>

#include "robocup3ds/Agent.hh"
#include "robocup3ds/GameState.hh"
#include "robocup3ds/Perceptors.hh"
#include "robocup3ds/Server.hh"
#include "robocup3ds/states/State.hh"
#include "robocup3ds/SoccerField.hh"

using namespace ignition;

int          Perceptor::updateVisualFreq  = 3;
bool         Perceptor::useNoise          = true;
const double Perceptor::kDistNoiseScale   = 0.01;
const double Perceptor::kHearDist         = 50.0;
const int    Perceptor::kUpdateHearFreq   = 2;

const math::Vector3<double> Perceptor::kFixedNoise(
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005),
  math::Rand::DblUniform(-0.005, 0.005));
const math::Vector3<double> Perceptor::kNoiseSigma(0.0965, 0.1225, 0.1480);

/////////////////////////////////////////////////
Perceptor::Perceptor(GameState *const _gameState):
  gameState(_gameState)
{
  this->SetViewFrustum(GameState::HFov, GameState::VFov);
}

/////////////////////////////////////////////////
void Perceptor::SetViewFrustum(const double _hfov, const double _vfov)
{
  double HFov = IGN_DTOR(std::min(180.0, std::max(0.0, _hfov)));
  double VFov = IGN_DTOR(std::min(180.0, std::max(0.0, _vfov)));

  math::Vector3<double> origin = math::Vector3<double>::Zero;
  math::Vector3<double> upperRight(1.0, -tan(HFov / 2), tan(VFov / 2));
  math::Vector3<double> upperLeft(1.0, tan(HFov / 2), tan(VFov / 2));
  math::Vector3<double> lowerRight(1.0, -tan(HFov / 2), -tan(VFov / 2));
  math::Vector3<double> lowerLeft(1.0, tan(HFov / 2), -tan(VFov / 2));

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
const
std::vector <ignition::math::Plane<double>> &Perceptor::GetViewFrustum() const
{
  return this->viewFrustum;
}

/////////////////////////////////////////////////
bool Perceptor::UpdatePerception() const
{
  return this->gameState->GetCycleCounter() % Perceptor::updateVisualFreq == 0;
}

/////////////////////////////////////////////////
Team::Side Perceptor::SideToSpeak() const
{
  if (this->gameState->GetCycleCounter()
      % Perceptor::kUpdateHearFreq == 0)
  {
    return Team::Side::LEFT;
  }
  else
  {
    // case where the following condition below is true
    // this->gameState->GetCycleCounter() % Perceptor::updateHearFreq == 1
    return Team::Side::RIGHT;
  }
}

/////////////////////////////////////////////////
void Perceptor::Update()
{
  for (auto &team : this->gameState->teams)
  {
    for (auto &agent : team->members)
    {
      this->SetG2LMat(agent);
      if (this->UpdatePerception())
      {
        // update line info
        agent.percept.fieldLines.clear();
        for (auto &fieldLine : SoccerField::kFieldLines)
        {
          this->UpdateLine(agent, fieldLine);
        }

        // update landmark info
        agent.percept.landMarks.clear();
        for (auto &kv : SoccerField::kLandMarks)
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
            {
              this->UpdateOtherAgent(agent, otherAgent);
            }
          }
        }
      }
      // update agent hear
      this->UpdateAgentHear(agent);
    }
  }

  // after calling UpdateAgentHear() for all agents, set team say to false
  for (const auto &team : this->gameState->teams)
  {
    if (this->SideToSpeak() == team->side)
    {
      team->say.isValid = false;
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
      {
        return;
      }
    }
  }

  agentLine.Set(addNoise(Geometry::CartToSphere(agentLine[0])),
                addNoise(Geometry::CartToSphere(agentLine[1])));
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
      {
        return;
      }
    }
  }

  _agent.percept.landMarks[_landmarkname] =
    addNoise(Geometry::CartToSphere(_agentLandMark));
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
        {
          return;
        }
      }
    }

    AgentId otherAgentId(_otherAgent.uNum,
                         _otherAgent.team->name);
    _agent.percept.otherAgentBodyMap[otherAgentId][kv.first] =
      addNoise(Geometry::CartToSphere(_otherAgentPart));
  }
}

/////////////////////////////////////////////////
void Perceptor::UpdateAgentHear(Agent &_agent) const
{
  const AgentSay *say = NULL;
  for (const auto &team : this->gameState->teams)
  {
    if (this->SideToSpeak() == team->side)
    {
      say = &team->say;
    }
  }

  AgentHear &hear = _agent.percept.hear;
  hear.isValid = false;
  if (!say || !say->isValid)
  {
    return;
  }
  const math::Vector3<double> relPos = this->G2LMat.TransformAffine(say->pos);
  if (relPos.Length() > Perceptor::kHearDist)
  {
    return;
  }

  hear.isValid = true;
  hear.gameTime = gameState->GetElapsedGameTime();
  hear.yaw = atan2(relPos.Y(), relPos.X());
  hear.self = (say->agentId == AgentId(_agent.uNum, _agent.team->name));
  hear.msg = say->msg;
}

/////////////////////////////////////////////////
int Perceptor::Serialize(const Agent &_agent, char *_string,
                         const int _size) const
{
  int cx = 0;

  // write out gamestate info and time
  int sl;
  int sr;
  for (const auto &team : this->gameState->teams)
  {
    if (team->side == Team::Side::LEFT)
    {
      sl = team->score;
    }
    if (team->side == Team::Side::RIGHT)
    {
      sr = team->score;
    }
  }

  //write out basic gamestate information
  cx += snprintf(_string + cx, _size - cx,
                 "(time (now %.2f)) (GS (unum %d) (team %s) "
                 "(t %.2f) (pm %s) (sl %d) (sr %d))",
                 this->gameState->GetGameTime(),
                 _agent.uNum,
                 Team::GetSideAsString(_agent.team->side).c_str(),
                 this->gameState->GetElapsedGameTime(true),
                 this->gameState->GetCurrentState()->name.c_str(),
                 sl, sr);

  // write hear info
  if (_agent.percept.hear.isValid && _agent.percept.hear.self)
  {
    cx += snprintf(_string + cx, _size - cx, "(hear %.2f self %s)",
                   _agent.percept.hear.gameTime,
                   _agent.percept.hear.msg.c_str());
  }
  else if (_agent.percept.hear.isValid)
  {
    cx += snprintf(_string + cx, _size - cx, "(hear %.2f %.2f %s)",
                   _agent.percept.hear.gameTime,
                   _agent.percept.hear.yaw,
                   _agent.percept.hear.msg.c_str());
  }

  // write body joint angle info
  for (const auto &kv : _agent.percept.hingeJoints)
  {
    cx += snprintf(_string + cx, _size - cx, " (HJ (n %s) (ax %.2f))",
                   kv.first.c_str(), kv.second);
  }

  // write out gyro info
  cx += snprintf(_string + cx, _size - cx,
                 "(GYR (n torso) (rt %.2f %.2f %.2f))",
                 _agent.percept.gyroRate.X(), _agent.percept.gyroRate.Y(),
                 _agent.percept.gyroRate.Z());

  // write out acceleration info
  cx += snprintf(_string + cx, _size - cx, "(ACC (n torso) (a %.2f %.2f %.2f))",
                 _agent.percept.accel.X(), _agent.percept.accel.Y(),
                 _agent.percept.accel.Z());

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

  if (this->UpdatePerception())
  {
    // write out perception info
    cx += snprintf(_string + cx, _size - cx, "(See");

    // write out landmark info
    for (const auto &kv : _agent.percept.landMarks)
    {
      cx += SerializePoint(kv.first.c_str(), kv.second,
                           _string + cx, _size - cx);
    }

    // write out other agent body parts
    for (const auto &kv : _agent.percept.otherAgentBodyMap)
    {
      cx += snprintf(_string + cx, _size - cx, " (P (team %s) (id %d)",
                     kv.first.second.c_str(), kv.first.first);
      for (const auto &kv2 : kv.second)
      {
        cx += SerializePoint(kv2.first.c_str(),
                             kv2.second, _string + cx, _size - cx);
      }
      cx += snprintf(_string + cx, _size - cx, ")");
    }

    // write out fieldlines
    for (const auto &fieldLine : _agent.percept.fieldLines)
    {
      cx += snprintf(_string + cx, _size - cx,
                     " (L (pol %.2f %.2f %.2f) (pol %.2f %.2f %.2f))",
                     fieldLine[0].X(), fieldLine[0].Y(), fieldLine[0].Z(),
                     fieldLine[1].X(), fieldLine[1].Y(), fieldLine[1].Z());
    }

    // finish writing out perception info
    cx += snprintf(_string + cx, _size - cx, ")");
  }

  // write out ground truth information
  if (GameState::groundTruthInfo)
  {
    const auto &ballPos = this->gameState->GetBall();
    if (_agent.team->side == Team::Side::LEFT)
    {
      cx += snprintf(_string + cx, _size - cx,
                     " (mypos %.2f %.2f %.2f) (myorien %.2f)"
                     " (ballpos %.2f %.2f %.2f)",
                     _agent.pos.X(), _agent.pos.Y(), _agent.pos.Z(),
                     IGN_RTOD(_agent.rot.Euler().Z()),
                     ballPos.X(), ballPos.Y(), ballPos.Z());
    }
    else
    {
      double rot = fmod(IGN_RTOD(_agent.rot.Euler().Z()) + 180.0, 360.0);
      cx += snprintf(_string + cx, _size - cx,
                     " (mypos %.2f %.2f %.2f) (myorien %.2f)"
                     " (ballpos %.2f %.2f %.2f)",
                     -_agent.pos.X(), -_agent.pos.Y(), _agent.pos.Z(),
                     rot, -ballPos.X(), -ballPos.Y(), ballPos.Z());
    }
  }

  return cx;
}

int Perceptor::SerializePoint(const char *_label,
                              const math::Vector3<double> &_pt,
                              char *_string, const int _size) const
{
  return snprintf(_string, _size, " (%s (pol %.2f %.2f %.2f))",
                  _label, _pt.X(), _pt.Y(), _pt.Z());
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
    _pt.X() + Perceptor::kFixedNoise.X() +
    math::Rand::DblNormal(0, Perceptor::kNoiseSigma.X())
    * _pt.X() * Perceptor::kDistNoiseScale,
    _pt.Y() + Perceptor::kFixedNoise.Y() +
    math::Rand::DblNormal(0, Perceptor::kNoiseSigma.Y()),
    _pt.Z() + Perceptor::kFixedNoise.Z() +
    math::Rand::DblNormal(0, Perceptor::kNoiseSigma.Z()));
  return newPt;
}

/////////////////////////////////////////////////
void Perceptor::SetG2LMat(const Agent &_agent)
{
  this->G2LMat = math::Matrix4<double>(_agent.cameraRot);
  this->G2LMat.Translate(_agent.cameraPos);
  this->G2LMat = this->G2LMat.Inverse();
}
