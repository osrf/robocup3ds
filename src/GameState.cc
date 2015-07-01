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
#include <array>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/Geometry.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/BeforeKickOffState.hh"
#include "robocup3ds/states/CornerKickLeftState.hh"
#include "robocup3ds/states/CornerKickRightState.hh"
#include "robocup3ds/states/FreeKickLeftState.hh"
#include "robocup3ds/states/FreeKickRightState.hh"
#include "robocup3ds/states/GameOverState.hh"
#include "robocup3ds/states/GoalKickLeftState.hh"
#include "robocup3ds/states/GoalKickRightState.hh"
#include "robocup3ds/states/GoalLeftState.hh"
#include "robocup3ds/states/GoalRightState.hh"
#include "robocup3ds/states/KickInLeftState.hh"
#include "robocup3ds/states/KickInRightState.hh"
#include "robocup3ds/states/KickOffLeftState.hh"
#include "robocup3ds/states/KickOffRightState.hh"
#include "robocup3ds/states/PlayOnState.hh"

using namespace ignition;
using namespace states;

const std::string GameState::BeforeKickOff   = "BeforeKickOff";
const std::string GameState::KickOffLeft     = "KickOffLeft";
const std::string GameState::KickOffRight    = "KickOffRight";
const std::string GameState::PlayOn          = "PlayOn";
const std::string GameState::KickInLeft      = "KickInLeft";
const std::string GameState::KickInRight     = "KickInRight";
const std::string GameState::CornerKickLeft  = "CornerKickLeft";
const std::string GameState::CornerKickRight = "CornerKickRight";
const std::string GameState::GoalKickLeft    = "GoalKickLeft";
const std::string GameState::GoalKickRight   = "GoalKickRight";
const std::string GameState::GameOver        = "GameOver";
const std::string GameState::GoalLeft        = "GoalLeft";
const std::string GameState::GoalRight       = "GoalRight";
const std::string GameState::FreeKickLeft    = "FreeKickLeft";
const std::string GameState::FreeKickRight   = "FreeKickRight";

double GameState::SecondsFullGame = 600;
double GameState::SecondsEachHalf = SecondsFullGame * 0.5;
double GameState::SecondsGoalPause = 3;
double GameState::SecondsKickInPause = 1;
double GameState::SecondsKickIn = 15;
double GameState::SecondsBeforeKickOff = 5;
double GameState::SecondsKickOff = 15;
bool   GameState::useCounterForGameTime = true;
int    GameState::playerLimit = 11;
int    GameState::penaltyBoxLimit = 3;
double GameState::beamHeight = SoccerField::RobotPoseHeight + 0.05;
double GameState::crowdingEnableRadius = 0.8;
double GameState::innerCrowdingRadius = 0.4;
double GameState::outerCrowdingRadius = 1.0;
double GameState::immobilityTimeLimit = 15;
double GameState::fallenTimeLimit = 30;
double GameState::dropBallRadius = 2;
double GameState::HFov = 120;
double GameState::VFov = 120;
bool   GameState::restrictVision = true;

/////////////////////////////////////////////////
GameState::GameState():
  beforeKickOffState(std::make_shared<BeforeKickOffState>(BeforeKickOff, this)),
  kickOffLeftState(std::make_shared<KickOffLeftState>(KickOffLeft, this)),
  kickOffRightState(std::make_shared<KickOffRightState>(KickOffRight, this)),
  playOnState(std::make_shared<PlayOnState>(PlayOn, this)),
  kickInLeftState(std::make_shared<KickInLeftState>(KickInLeft, this)),
  kickInRightState(std::make_shared<KickInRightState>(KickInRight, this)),
  cornerKickLeftState(std::make_shared<CornerKickLeftState>(
                        CornerKickLeft, this)),
  cornerKickRightState(std::make_shared<CornerKickRightState>(
                         CornerKickRight, this)),
  goalKickLeftState(std::make_shared<GoalKickLeftState>(GoalKickLeft, this)),
  goalKickRightState(std::make_shared<GoalKickRightState>(GoalKickRight, this)),
  gameOverState(std::make_shared<GameOverState>(GameOver, this)),
  goalLeftState(std::make_shared<GoalLeftState>(GoalLeft, this)),
  goalRightState(std::make_shared<GoalRightState>(GoalRight, this)),
  freeKickLeftState(std::make_shared<FreeKickLeftState>(FreeKickLeft, this)),
  freeKickRightState(std::make_shared<FreeKickRightState>(FreeKickRight, this)),
  touchBallKickoff(std::shared_ptr<BallContact>(nullptr)),
  currentState(std::shared_ptr<State>(nullptr))
{
  this->half = Half::FIRST_HALF;
  this->cycleCounter = 0;
  this->gameTime = this->prevCycleGameTime = this->startGameTime = 0.0;
  this->updateBallPose = false;
  this->hasCurrentStateChanged = false;
  this->SetCurrent(beforeKickOffState);
  this->teams.push_back(
    std::shared_ptr<Team>(
      new Team("_empty_team", Team::Side::LEFT, 0, GameState::playerLimit)));
  this->teams.push_back(
    std::shared_ptr<Team>(
      new Team("_empty_team", Team::Side::RIGHT, 0, GameState::playerLimit)));
}

/////////////////////////////////////////////////
GameState::~GameState()
{
}

/////////////////////////////////////////////////
void GameState::LoadConfiguration(
  const std::map<std::string, std::string> &_config) const
{
  double value;
  bool boolValue;
  if (LoadConfigParameter(_config, "gamestate_secondsfullgame", value))
  {
    GameState::SecondsFullGame = value;
    GameState::SecondsEachHalf = 0.5 * GameState::SecondsFullGame;
  }
  else if (LoadConfigParameter(_config, "gamestate_secondseachhalf", value))
  {
    GameState::SecondsEachHalf = value;
    GameState::SecondsFullGame = 2.0 * GameState::SecondsEachHalf;
  }
  if (LoadConfigParameter(_config, "gamestate_secondsgoalpause", value))
  { GameState::SecondsGoalPause = value; }
  if (LoadConfigParameter(_config, "gamestate_secondskickinpause", value))
  { GameState::SecondsKickInPause = value; }
  if (LoadConfigParameter(_config, "gamestate_secondskickin", value))
  { GameState::SecondsKickIn = value; }
  if (LoadConfigParameter(_config, "gamestate_secondsbeforekickoff", value))
  { GameState::SecondsBeforeKickOff = value; }
  if (LoadConfigParameter(_config, "gamestate_secondskickoff", value))
  { GameState::SecondsKickOff = value; }
  if (LoadConfigParameter(_config, "gamestate_dropballradius", value))
  { GameState::dropBallRadius = value; }
  if (LoadConfigParameterBool(
    _config, "gamestate_usecounterforgametime", boolValue))
  { GameState::useCounterForGameTime = boolValue; }
  if (LoadConfigParameter(_config, "gamestate_playerlimit", value))
  { GameState::playerLimit = static_cast<int>(value); }
  if (LoadConfigParameter(_config, "gamestate_penaltyboxlimit", value))
  { GameState::penaltyBoxLimit = static_cast<int>(value); }
  if (LoadConfigParameter(_config, "gamestate_beamheight", value))
  { GameState::beamHeight = value; }
  if (LoadConfigParameter(_config, "gamestate_crowdingenableradius", value))
  { GameState::crowdingEnableRadius = value; }
  if (LoadConfigParameter(_config, "gamestate_innercrowdingradius", value))
  { GameState::innerCrowdingRadius = value; }
  if (LoadConfigParameter(_config, "gamestate_outercrowdingradius", value))
  { GameState::outerCrowdingRadius = value; }
  if (LoadConfigParameter(_config, "gamestate_immobilitytimelimit", value))
  { GameState::immobilityTimeLimit = value; }
  if (LoadConfigParameter(_config, "gamestate_fallentimelimit", value))
  { GameState::fallenTimeLimit = value; }
  if (LoadConfigParameter(_config, "percept_hfov", value))
  { GameState::HFov = value; }
  if (LoadConfigParameter(_config, "percept_vfov", value))
  { GameState::VFov = value; }
  if (LoadConfigParameterBool(_config, "percept_restrictvision", boolValue))
  { GameState::restrictVision = boolValue; }
}

/////////////////////////////////////////////////
bool GameState::LoadConfigParameter(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  double &_value) const
{
  try
  {
    _value = std::stod(_config.at(_key));
  }
  catch (const std::exception &exc)
  {
    std::cerr << exc.what();
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool GameState::LoadConfigParameterBool(
  const std::map<std::string, std::string> &_config,
  const std::string &_key,
  bool &_boolValue) const
{
  try
  {
    _boolValue = _config.at(_key) != "false";
  }
  catch (const std::exception &exc)
  {
    std::cerr << exc.what();
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void GameState::ClearBallContactHistory()
{
  this->ballContactHistory.clear();
}

/////////////////////////////////////////////////
void GameState::Update()
{
  this->cycleCounter++;
  if (GameState::useCounterForGameTime)
  {
    this->gameTime = this->cycleCounter * 0.02;
  }

  this->hasCurrentStateChanged = false;
  if (this->currentState)
  {
    this->currentState->Update();
  }

  this->prevCycleGameTime = this->gameTime;
}

/////////////////////////////////////////////////
void GameState::ReleasePlayers()
{
  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      agent.status = Agent::Status::RELEASED;
    }
  }
}

/////////////////////////////////////////////////
void GameState::StopPlayers()
{
  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      agent.status = Agent::Status::STOPPED;
    }
  }
}

////////////////////////////////////////////////
void GameState::SetCurrent(const std::shared_ptr<State> &_newState,
                           const bool _resetState)
{
  // Only update the state if _newState is different than the current state.
  if (this->currentState != _newState || _resetState)
  {
    this->Initialize();
    if (this->currentState != NULL)
    {
      this->currentState->Uninitialize();
    }
    _newState->prevState = this->currentState;
    this->currentState = _newState;
    this->currentState->Preinitialize();
    this->hasCurrentStateChanged = true;
  }
}

/////////////////////////////////////////////////
void GameState::DropBallImpl(const Team::Side _teamAllowed)
{
  // Check if the player is withing FREE_KICK distance.
  for (auto &team : this->teams)
  {
    if (team->side != _teamAllowed)
    {
      for (auto &agent : team->members)
      {
        // Move the player if it's close enough to the ball.
        if (agent.pos.Distance(ballPos) < GameState::dropBallRadius)
        {
          // Calculate the general form equation of a line from two points.
          // a = y1 - y2
          // b = x2 - x1
          // c = (x1-x2)*y1 + (y2-y1)*x1
          math::Vector3<double> v(ballPos.Y() - agent.pos.Y(),
                                  agent.pos.X() - ballPos.X(),
                                  (ballPos.X() - agent.pos.X()) * ballPos.Y() +
                                  (agent.pos.Y() - ballPos.Y()) * ballPos.X());
          math::Vector3<double> newPos;
          newPos.Set(0, 0, beamHeight);
          math::Vector3<double> newPos2;
          newPos2.Set(0, 0, beamHeight);
          if (Geometry::IntersectionCircunferenceLine(v, ballPos, 1.25 *
              GameState::dropBallRadius, newPos, newPos2))
          {
            if (agent.pos.Distance(newPos) < agent.pos.Distance(newPos2))
            {
              this->MoveAgent(agent, newPos);
            }
            else
            {
              this->MoveAgent(agent, newPos2);
            }
          }
        }
      }
    }
  }
}


/////////////////////////////////////////////////
void GameState::CheckTiming()
{
  if (this->hasCurrentStateChanged)
  {
    return;
  }

  double elapsedGameTime = this->GetElapsedGameTime();
  if ((this->half == Half::FIRST_HALF) && (elapsedGameTime >= SecondsEachHalf))
  {
    // swap team sides
    Team::Side temp = this->teams.at(0)->side;
    this->teams.at(0)->side = this->teams.at(1)->side;
    this->teams.at(1)->side = temp;

    // End of the first half
    this->startGameTime = this->gameTime;
    this->SetHalf(Half::SECOND_HALF);
    this->SetCurrent(kickOffRightState);
  }
  else if ((this->half == Half::SECOND_HALF)
           && (elapsedGameTime >= SecondsEachHalf))
  {
    // End of the game
    this->SetCurrent(gameOverState);
  }
}

/////////////////////////////////////////////////
void GameState::CheckBall()
{
  if (this->hasCurrentStateChanged)
  {
    return;
  }
  Team::Side lastContactSide;
  if (this->ballContactHistory.size() > 0)
  {
    lastContactSide = this->GetLastBallContact()->side;
  }
  else
  {
    if (this->half == Half::FIRST_HALF)
    {
      lastContactSide = Team::Side::LEFT;
    }
    else
    {
      lastContactSide = Team::Side::RIGHT;
    }
  }

  // The ball is inside the left goal.
  if (SoccerField::GoalBoxLeft.Contains(this->ballPos))
  {
    this->SetCurrent(goalRightState);
  }
  else if (SoccerField::GoalBoxRight.Contains(this->ballPos))
  {
    // The ball is inside the right goal.
    this->SetCurrent(goalLeftState);
  }
  else if (fabs(this->ballPos.Y()) > SoccerField::HalfFieldHeight +
           SoccerField::OutofBoundsTol)
  {
    // The ball is outside of the sideline.
    // Choose team
    if (lastContactSide == Team::Side::LEFT)
    { this->SetCurrent(kickInRightState); }
    else
    { this->SetCurrent(kickInLeftState); }
  }
  else if (fabs(this->ballPos.X()) > SoccerField::HalfFieldWidth +
           SoccerField::OutofBoundsTol)
  {
    // The ball is outside of the field
    // over the defensive team's goal line.
    if (this->ballPos.X() < 0)
    {
      // Choose team 1
      if (lastContactSide == Team::Side::LEFT)
      { this->SetCurrent(cornerKickRightState); }
      else
      { this->SetCurrent(goalKickLeftState); }
    }
    else
    {
      // Choose team 2
      if (lastContactSide == Team::Side::LEFT)
      { this->SetCurrent(goalKickRightState); }
      else
      { this->SetCurrent(cornerKickLeftState); }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckGoalKickIllegalDefense(const Team::Side _teamAllowed)
{
  math::Box penaltyBox;
  if (_teamAllowed == Team::Side::LEFT)
  {
    penaltyBox = SoccerField::PenaltyBoxLeft;
  }
  else
  {
    penaltyBox = SoccerField::PenaltyBoxRight;
  }

  for (auto &team : this->teams)
  {
    if (team->side != _teamAllowed)
    {
      for (auto &agent : team->members)
      {
        if (penaltyBox.Contains(agent.pos))
        {
          this->MoveAgentToSide(agent);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckIllegalDefense()
{
  math::Box penaltyBox;
  math::Vector3<double> goalCenter;
  for (auto &team : this->teams)
  {
    if (team->side == Team::Side::LEFT)
    {
      penaltyBox = SoccerField::PenaltyBoxLeft;
      goalCenter = SoccerField::GoalCenterLeft;
    }
    else
    {
      penaltyBox = SoccerField::PenaltyBoxRight;
      goalCenter = SoccerField::GoalCenterRight;
    }

    // do bookkeeping for agents that leave penalty box
    for (auto &agent : team->members)
    {
      if (!penaltyBox.Contains(agent.pos) && agent.inPenaltyBox)
      {
        agent.inPenaltyBox = false;
        team->numPlayersInPenaltyBox--;
      }
    }

    // do bookkeeping for agents that enter penalty box
    for (auto &agent : team->members)
    {
      if (!agent.inPenaltyBox)
      {
        if (penaltyBox.Contains(agent.pos) && team->numPlayersInPenaltyBox <
            GameState::penaltyBoxLimit)
        {
          // account for agent if there is still room in penalty box
          team->numPlayersInPenaltyBox++;
          agent.inPenaltyBox = true;
        }
        else if (penaltyBox.Contains(agent.pos) && team->
                 numPlayersInPenaltyBox >= GameState::penaltyBoxLimit)
        {
          // if agent is not goalie: move agent away if penalty box is
          // already crowded
          // if agent is goalie: move another agent that is farthest
          // from goal away
          if (agent.IsGoalKeeper())
          {
            double bestDist = -1;
            Agent *bestAgent = NULL;
            for (auto &nonGoalieAgent : team->members)
            {
              if (nonGoalieAgent.IsGoalKeeper())
              {
                continue;
              }
              double nonGoalieAgentDist = nonGoalieAgent.pos.Distance(
                                            goalCenter);
              if (nonGoalieAgent.inPenaltyBox && nonGoalieAgentDist
                  > bestDist)
              {
                bestDist = nonGoalieAgentDist;
                bestAgent = &nonGoalieAgent;
              }
            }
            if (bestAgent != NULL)
            {
              this->MoveAgentToSide(*bestAgent);
            }
            agent.inPenaltyBox = true;
          }
          else
          {
            this->MoveAgentToSide(agent);
          }
        }
        else
        {
          // agent is not penalty box nor is accounted for so do nothing
        }
      }
    }
  }
}

/////////////////////////////////////////////////
bool GameState::SortDist(const AgentDist &_i, const AgentDist &_j)
{
  return (_i.dist < _j.dist);
}

/////////////////////////////////////////////////
void GameState::CheckCrowding()
{
  bool enableCrowding = false;
  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      if (agent.pos.Distance(ballPos) < GameState::crowdingEnableRadius)
      {
        enableCrowding = true;
        goto exitLoop;
      }
    }
  }
exitLoop:

  if (enableCrowding)
  {
    for (auto &team : this->teams)
    {
      std::vector<AgentDist> agentDists;
      for (auto &agent : team->members)
      {
        AgentDist agentDist;
        agentDist.agent = &agent;
        agentDist.dist = agentDist.agent->pos.Distance(ballPos);
        agentDists.push_back(agentDist);
      }
      std::sort(agentDists.begin(), agentDists.end(), SortDist);

      // only allow one agent to be in inner radius
      int reposition_2 = 1;
      for (auto &agentDist : agentDists)
      {
        if (agentDist.dist < GameState::innerCrowdingRadius)
        {
          if (reposition_2 > 0)
          {
            reposition_2--;
          }
          else
          {
            this->MoveAgentToSide(*agentDist.agent);
          }
        }
      }
      // only allow two agents to be in outer radius
      int reposition_3 = 2;
      for (auto &agentDist : agentDists)
      {
        if (agentDist.dist < GameState::outerCrowdingRadius)
        {
          if (reposition_3 > 0)
          {
            reposition_3--;
          }
          else
          {
            this->MoveAgentToSide(*agentDist.agent);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckImmobility()
{
  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      if (agent.pos == agent.prevPos)
      {
        agent.timeImmoblized += this->GetElapsedCycleGameTime();
      }
      else
      {
        agent.timeImmoblized = 0;
      }

      if (agent.pos.Z() < SoccerField::RobotPoseHeight * 0.5)
      {
        agent.timeFallen += this->GetElapsedCycleGameTime();
      }
      else
      {
        agent.timeFallen = 0;
      }

      // move agent to side of field if they have remained fallen
      // or timeout too long.
      double SCALE = 1.0 + (agent.uNum == 1);
      if (agent.timeImmoblized >= SCALE * GameState::immobilityTimeLimit
          || agent.timeFallen >= SCALE * GameState::fallenTimeLimit)
      {
        agent.timeImmoblized = 0;
        agent.timeFallen = 0;
        this->MoveAgentToSide(agent);
      }
      agent.prevPos = agent.pos;
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckDoubleTouch()
{
  if (this->ballContactHistory.size() != 2 ||
      this->hasCurrentStateChanged)
  {
    return;
  }

  // check && make sure that the first contact after kick off
  // (or second overall contact) is not by the same agent who performed
  // the kick off
  std::shared_ptr<BallContact> firstContact = this->ballContactHistory.at(1);
  if (this->touchBallKickoff != NULL
      && this->currentState->prevState != NULL
      && (this->currentState->prevState->GetName() == "KickOffRight"
          || this->currentState->prevState->GetName() == "KickOffLeft")
      && this->touchBallKickoff->side == firstContact->side
      && this->touchBallKickoff->uNum == firstContact->uNum)
  {
    if (this->currentState->prevState->GetName() == "KickOffLeft")
    {
      this->SetCurrent(kickOffRightState);
    }
    else
    {
      this->SetCurrent(kickOffLeftState);
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckCanScore()
{
  std::shared_ptr<BallContact> ballContact = this->GetLastBallContact();
  if (ballContact == NULL)
  {
    return;
  }
  for (auto &team : this->teams)
  {
    if ((!team->canScore)
        && (this->touchBallKickoff != NULL)
        && ((ballContact->side != team->side)
            || (ballContact->side == team->side
                && this->touchBallKickoff->uNum != ballContact->uNum
                && ballContact->contactPos.Distance(
                  SoccerField::CenterOfField) >
                SoccerField::CenterCircleRadius)))
    {
      team->canScore = true;
    }
  }
}

void GameState::CheckOffSidesOnKickOff(const Team::Side _kickingSide)
{
  // check for agents that violate sides
  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      math::Vector3<double> agentPosNoZ(agent.pos.X(), agent.pos.Y(), 0);

      bool isOffSide;
      if (team->side == Team::Side::LEFT)
      { isOffSide = agent.pos.X() > 0; }
      else
      { isOffSide = agent.pos.X() < 0; }

      // if on kicking team, must stay in circle or cannot cross line.
      if (team->side == _kickingSide && (isOffSide &&
                                         agentPosNoZ.Distance(
                                           SoccerField::CenterOfField) >
                                         SoccerField::CenterCircleRadius))
      {
        // move them to side of field for now
        this->MoveOffSideAgent(agent);

        // if on defending team, cannot cross line and go inside circle.
      }
      else if (team->side != _kickingSide &&
               (isOffSide || agentPosNoZ.Distance(SoccerField::CenterOfField)
                < SoccerField::CenterCircleRadius))
      {
        // move them to side of field for now
        this->MoveOffSideAgent(agent);
      }
    }
  }
}


/////////////////////////////////////////////////
void GameState::MoveAgent(Agent &_agent,
                          const math::Vector3<double> &_pos) const
{
  _agent.pos = _pos;
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgent(Agent &_agent, const double _x, const double _y,
                          const double _yaw) const
{
  _agent.pos.Set(_x, _y, GameState::beamHeight);
  _agent.rot.Euler(0, 0, _yaw);
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                               const double _yaw) const
{
  double offsetX = (static_cast<double>(random()) / (RAND_MAX)) * 0.2 - 0.1;
  double offsetY = (static_cast<double>(random()) / (RAND_MAX)) *
                   0.2 - 0.1;
  double offsetYaw = (static_cast<double>(random()) / (RAND_MAX)) *
                     0.2 - 0.1;
  _agent.pos.Set(_x + offsetX, _y + offsetY, GameState::beamHeight);
  _agent.rot.Euler(0, 0, _yaw + offsetYaw);
  _agent.updatePose = true;
}

void GameState::MoveAgent(Agent &_agent, const math::Vector3<double> &_pos,
                          const math::Quaternion<double> &_rot) const
{
  _agent.pos = _pos;
  _agent.rot = _rot;
  _agent.updatePose = true;
}

void GameState::MoveAgentToSide(Agent &_agent) const
{
  double newY;
  if (_agent.pos.Y() > 0)
  {
    newY = -SoccerField::HalfFieldHeight;
  }
  else
  {
    newY = SoccerField::HalfFieldHeight;
  }
  _agent.pos = math::Vector3<double>(_agent.pos.X(), newY,
                                     GameState::beamHeight);
  _agent.updatePose = true;
}

void GameState::MoveOffSideAgent(Agent &_agent) const
{
  if (_agent.team->side == Team::Side::LEFT)
  {
    _agent.pos.Set(-(SoccerField::CenterCircleRadius + 0.5), _agent.pos.Y(),
                   GameState::beamHeight);
  }
  else if (_agent.team->side == Team::Side::RIGHT)
  {
    _agent.pos.Set(SoccerField::CenterCircleRadius + 0.5, _agent.pos.Y(),
                   GameState::beamHeight);
  }
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
math::Vector3<double> GameState::GetBall()
{
  return this->ballPos;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCenter()
{
  this->ballPos = SoccerField::BallCenterPosition;
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallForGoalKick()
{
  double newX = SoccerField::HalfFieldWidth - 1.0;
  if (ballPos.X() < 0)
  {
    newX = -newX;
  }
  this->ballPos = math::Vector3<double>(newX, 0, SoccerField::BallRadius);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCorner()
{
  this->ballPos = math::Vector3<double>(
                    (fabs(ballPos.X()) / ballPos.X()) *
                    SoccerField::HalfFieldWidth,
                    (fabs(ballPos.Y()) / ballPos.Y()) *
                    SoccerField::HalfFieldHeight,
                    SoccerField::BallRadius);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallInBounds()
{
  double newX = std::max(std::min(SoccerField::HalfFieldWidth, ballPos.X()),
                         -SoccerField::HalfFieldWidth);
  double newY = std::max(std::min(SoccerField::HalfFieldHeight, ballPos.Y()),
                         -SoccerField::HalfFieldHeight);
  this->ballPos.Set(newX, newY, SoccerField::BallRadius);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBall(const math::Vector3<double> &_ballPos)
{
  this->ballPos = _ballPos;
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::SetBallVel(const math::Vector3<double> &_ballVel)
{
  this->ballVel = _ballVel;
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::SetBallAngVel(const math::Vector3<double> &_ballAngVel)
{
  this->ballAngVel = _ballAngVel;
  this->updateBallPose = true;
}

////////////////////////////////////////////////
void GameState::Initialize()
{
  // do something when play mode changes
  // for now, do nothing
}

////////////////////////////////////////////////
bool GameState::AddAgent(const int _uNum, const std::string &_teamName)
{
  int uNum = _uNum;
  // std::cout << "adding agent: " << uNum << " teamName: "
  // << teamName << std::endl;
  if (uNum < 0 || uNum > GameState::playerLimit)
  {
    // std::cout << "uNum " << uNum << " is invalid, cannot add agent to team "
    // << teamName << "!" << std::endl;
    return false;
  }
  std::shared_ptr<Team> teamToAdd(nullptr);
  for (auto &team : this->teams)
  {
    if (team->name == _teamName)
    {
      teamToAdd = team;
    }
  }
  if (!teamToAdd)
  {
    for (auto &team : this->teams)
    {
      if (team->name == "_empty_team" && team->members.size() == 0)
      {
        teamToAdd = team;
        team->name = _teamName;
        break;
      }
    }
  }
  if (!teamToAdd)
  {
    // std::cout << uNum << " " << teamName.c_str() << std::endl;
    // std::cout << "There already are two teams, cannot add agent into
    // new team!" << std::endl;
    return false;
  }

  // code below never reached
  // if (static_cast<int>(teamToAdd->members.size()) > GameState::playerLimit)
  // {
  //   return false;
  // }

  std::vector<bool> uNumArray;
  for (int i = 0; i < GameState::playerLimit; ++i)
  {
    uNumArray.push_back(true);
  }
  for (const auto &agent : teamToAdd->members)
  {
    uNumArray.at(agent.uNum - 1) = false;
    if (uNum != 0 && agent.uNum == uNum)
    {
      // std::cout << "Already have an agent with this unum: " << uNum <<
      // ", cannot add agent to team!" << std::endl;
      return false;
    }
  }
  if (uNum == 0)
  {
    for (int i = 0; i < GameState::playerLimit; ++i)
    {
      if (uNumArray.at(i))
      {
        uNum = i + 1;
        break;
      }
    }
  }
  if (uNum == 0)
  {
    // std::cout << "No free uNums avaliable, cannot add agent to team!"
    // << std::endl;
    return false;
  }
  teamToAdd->members.push_back(Agent(uNum, teamToAdd));
  return true;
}


///////////////////////////////////////////////
bool GameState::RemoveAgent(const int _uNum, const std::string &_teamName)
{
  for (const auto &team : this->teams)
  {
    if (team->name == _teamName)
    {
      for (const auto &agent : team->members)
      {
        if (agent.uNum == _uNum)
        {
          team->members.erase(std::remove(team->members.begin(),
                                          team->members.end(), agent),
                              team->members.end());
          return true;
        }
      }
    }
  }
  // std::cout << "Agent not found, unable to remove agent!" << std::endl;
  return false;
}

////////////////////////////////////////////////
bool GameState::BeamAgent(const int _uNum, const std::string &_teamName,
                          const double _x, const double _y, const double _rot)
{
  if (this->currentState->GetName() != "BeforeKickOff"
      && this->currentState->GetName() != "GoalKickLeft"
      && this->currentState->GetName() != "GoalKickRight")
  {
    // std::cout << "Incorrect play mode, unable to beam agent!" << std::endl;
    return false;
  }
  for (const auto &team : this->teams)
  {
    if (team->name == _teamName)
    {
      for ( auto &agent : team->members)
      {
        if (agent.uNum == _uNum)
        {
          this->MoveAgentNoisy(agent, _x, _y, _rot);
          return true;
        }
      }
    }
  }
  // std::cout << "Agent not found, unable to beam agent!" << std::endl;
  return false;
}

////////////////////////////////////////////////
GameState::Half GameState::GetHalf() const
{
  return this->half;
}

////////////////////////////////////////////////
void GameState::SetHalf(const Half _newHalf)
{
  this->half = _newHalf;
}

////////////////////////////////////////////////
double GameState::GetElapsedGameTime() const
{
  return this->gameTime - this->startGameTime;
}

////////////////////////////////////////////////
double GameState::GetElapsedCycleGameTime() const
{
  return this->gameTime - this->prevCycleGameTime;
}

////////////////////////////////////////////////
void GameState::SetGameTime(const double _gameTime)
{
  this->gameTime = _gameTime;
}

////////////////////////////////////////////////
double GameState::GetGameTime() const
{
  return this->gameTime;
}

////////////////////////////////////////////////
double GameState::GetStartGameTime() const
{
  return this->startGameTime;
}

////////////////////////////////////////////////
void GameState::SetStartGameTime(const double _startGameTime)
{
  this->startGameTime = _startGameTime;
}

////////////////////////////////////////////////
void GameState::SetCycleCounter(const int _cycleCounter)
{
  this->cycleCounter = _cycleCounter;

  if (GameState::useCounterForGameTime)
  {
    double newGameTime = 0.02 * cycleCounter;
    double offsetTime = newGameTime - gameTime;
    this->prevCycleGameTime += offsetTime;
    this->gameTime = newGameTime;
  }
}

////////////////////////////////////////////////
int GameState::GetCycleCounter() const
{
  return this->cycleCounter;
}

////////////////////////////////////////////////
std::shared_ptr<State> GameState::GetCurrentState() const
{
  return this->currentState;
}

////////////////////////////////////////////////
GameState::Team::Side GameState::GetLastSideTouchedBall() const
{
  if (this->GetLastBallContact() != NULL)
  {
    return this->GetLastBallContact()->side;
  }
  else
  {
    return Team::Side::NEITHER;
  }
}

////////////////////////////////////////////////
std::shared_ptr<GameState::BallContact> GameState::GetLastBallContact() const
{
  if (this->ballContactHistory.size() > 0)
  {
    return this->ballContactHistory.at(
             this->ballContactHistory.size() - 1);
  }
  else
  {
    return std::shared_ptr<BallContact>(nullptr);
  }
}
