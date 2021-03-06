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
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#include <algorithm>
#include <iostream>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/Agent.hh"
#include "robocup3ds/Geometry.hh"
#include "robocup3ds/Nao.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/states/BeforeKickOffState.hh"
#include "robocup3ds/states/CornerKickState.hh"
#include "robocup3ds/states/FreeKickState.hh"
#include "robocup3ds/states/GameOverState.hh"
#include "robocup3ds/states/GoalKickState.hh"
#include "robocup3ds/states/GoalState.hh"
#include "robocup3ds/states/KickInState.hh"
#include "robocup3ds/states/KickOffState.hh"
#include "robocup3ds/states/PlayOnState.hh"
#include "robocup3ds/Util.hh"

using namespace ignition;
using namespace states;
using namespace Util;

const std::string GameState::BeforeKickOff   = "BeforeKickOff";
const std::string GameState::KickOffLeft     = "KickOff_Left";
const std::string GameState::KickOffRight    = "KickOff_Right";
const std::string GameState::PlayOn          = "PlayOn";
const std::string GameState::KickInLeft      = "KickIn_Left";
const std::string GameState::KickInRight     = "KickIn_Right";
const std::string GameState::CornerKickLeft  = "corner_kick_left";
const std::string GameState::CornerKickRight = "corner_kick_right";
const std::string GameState::GoalKickLeft    = "goal_kick_left";
const std::string GameState::GoalKickRight   = "goal_kick_right";
const std::string GameState::GameOver        = "GameOver";
const std::string GameState::GoalLeft        = "Goal_Left";
const std::string GameState::GoalRight       = "Goal_Right";
const std::string GameState::FreeKickLeft    = "free_kick_left";
const std::string GameState::FreeKickRight   = "free_kick_right";

double GameState::SecondsFullGame = 600;
double GameState::SecondsEachHalf = GameState::SecondsFullGame * 0.5;
double GameState::SecondsGoalPause = 3;
double GameState::SecondsKickInPause = 1;
double GameState::SecondsKickIn = 15;
double GameState::SecondsBeforeKickOff = -1;
double GameState::SecondsKickOff = 15;
bool   GameState::useCounterForGameTime = false;
int    GameState::playerLimit = 11;
int    GameState::penaltyBoxLimit = 3;
double GameState::beamHeightOffset = 0.05;
double GameState::crowdingEnableRadius = 0.8;
double GameState::innerCrowdingRadius = 0.4;
double GameState::outerCrowdingRadius = 1.0;
double GameState::immobilityTimeLimit = 15;
double GameState::fallenTimeLimit = 30;
double GameState::dropBallRadius = 2;
double GameState::HFov = 120;
double GameState::VFov = 120;
bool   GameState::restrictVision = true;
bool   GameState::groundTruthInfo = false;
const double GameState::kOnBallDist = 0.001;
const double GameState::kCounterCycleTime = 0.02;
const double GameState::kDropBallRadiusMargin = 0.5;
const double GameState::kBeamNoise = 0.1;
const std::string GameState::kDefaultTeamName = "------------";
const double GameState::kBallContactInterval = 0.1;

/////////////////////////////////////////////////
GameState::GameState():
  beforeKickOffState(std::make_shared<BeforeKickOffState>(BeforeKickOff, this)),
  kickOffLeftState(std::make_shared<KickOffState>(KickOffLeft, this,
                   Team::Side::LEFT)),
  kickOffRightState(std::make_shared<KickOffState>(KickOffRight, this,
                    Team::Side::RIGHT)),
  playOnState(std::make_shared<PlayOnState>(PlayOn, this)),
  kickInLeftState(std::make_shared<KickInState>(KickInLeft, this,
                  Team::Side::LEFT)),
  kickInRightState(std::make_shared<KickInState>(KickInRight, this,
                   Team::Side::RIGHT)),
  cornerKickLeftState(std::make_shared<CornerKickState>(
                        CornerKickLeft, this, Team::Side::LEFT)),
  cornerKickRightState(std::make_shared<CornerKickState>(
                         CornerKickRight, this, Team::Side::RIGHT)),
  goalKickLeftState(std::make_shared<GoalKickState>(GoalKickLeft, this,
                    Team::Side::LEFT)),
  goalKickRightState(std::make_shared<GoalKickState>(GoalKickRight, this,
                     Team::Side::RIGHT)),
  gameOverState(std::make_shared<GameOverState>(GameOver, this)),
  goalLeftState(std::make_shared<GoalState>(GoalLeft, this,
                Team::Side::LEFT)),
  goalRightState(std::make_shared<GoalState>(GoalRight, this,
                 Team::Side::RIGHT)),
  freeKickLeftState(std::make_shared<FreeKickState>(FreeKickLeft, this,
                    Team::Side::LEFT)),
  freeKickRightState(std::make_shared<FreeKickState>(FreeKickRight, this,
                     Team::Side::RIGHT)),
  agentBodyTypeMap(
{
  {"NaoOfficialBT", std::make_shared<NaoOfficialBT>()},
  {"NaoSimsparkBT", std::make_shared<NaoSimsparkBT>()}
}),
defaultBodyType(agentBodyTypeMap.at("NaoSimsparkBT")),
hasCurrentStateChanged(false),
touchBallKickoff(nullptr),
updateBallPose(false),
gameTime(0.0),
prevCycleGameTime(0.0),
lastCycleTimeLength(0.0),
startGameTime(0.0),
currentState(nullptr),
half(Half::FIRST_HALF),
cycleCounter(0)
{
  this->playModeNameMap[beforeKickOffState->name] = beforeKickOffState;
  this->playModeNameMap[kickOffLeftState->name] = kickOffLeftState;
  this->playModeNameMap[kickOffRightState->name] = kickOffRightState;
  this->playModeNameMap[playOnState->name] = playOnState;
  this->playModeNameMap[kickInLeftState->name] = kickInLeftState;
  this->playModeNameMap[kickInRightState->name] = kickInRightState;
  this->playModeNameMap[cornerKickLeftState->name] = cornerKickLeftState;
  this->playModeNameMap[cornerKickRightState->name] = cornerKickRightState;
  this->playModeNameMap[goalKickLeftState->name] = goalKickLeftState;
  this->playModeNameMap[goalKickRightState->name] = goalKickRightState;
  this->playModeNameMap[gameOverState->name] = gameOverState;
  this->playModeNameMap[goalLeftState->name] = goalLeftState;
  this->playModeNameMap[goalRightState->name] = goalRightState;
  this->playModeNameMap[freeKickLeftState->name] = freeKickLeftState;
  this->playModeNameMap[freeKickRightState->name] = freeKickRightState;
  this->SetCurrent(beforeKickOffState);

  this->teams.push_back(std::make_shared<Team>(
                          GameState::kDefaultTeamName, Team::Side::LEFT, 0,
                          GameState::playerLimit));
  this->teams.push_back(std::make_shared<Team>(
                          GameState::kDefaultTeamName, Team::Side::RIGHT, 0,
                          GameState::playerLimit));
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
  {
    GameState::SecondsGoalPause = value;
  }
  if (LoadConfigParameter(_config, "gamestate_secondskickinpause", value))
  {
    GameState::SecondsKickInPause = value;
  }
  if (LoadConfigParameter(_config, "gamestate_secondskickin", value))
  {
    GameState::SecondsKickIn = value;
  }
  if (LoadConfigParameter(_config, "gamestate_secondsbeforekickoff", value))
  {
    GameState::SecondsBeforeKickOff = value;
  }
  if (LoadConfigParameter(_config, "gamestate_secondskickoff", value))
  {
    GameState::SecondsKickOff = value;
  }
  if (LoadConfigParameter(_config, "gamestate_dropballradius", value))
  {
    GameState::dropBallRadius = value;
  }
  if (LoadConfigParameterBool(
        _config, "gamestate_usecounterforgametime", boolValue))
  {
    GameState::useCounterForGameTime = boolValue;
  }
  if (LoadConfigParameter(_config, "gamestate_playerlimit", value))
  {
    GameState::playerLimit = static_cast<int>(value);
  }
  if (LoadConfigParameter(_config, "gamestate_penaltyboxlimit", value))
  {
    GameState::penaltyBoxLimit = static_cast<int>(value);
  }
  if (LoadConfigParameter(_config, "gamestate_beamheightoffset", value))
  {
    GameState::beamHeightOffset = value;
  }
  if (LoadConfigParameter(_config, "gamestate_crowdingenableradius", value))
  {
    GameState::crowdingEnableRadius = value;
  }
  if (LoadConfigParameter(_config, "gamestate_innercrowdingradius", value))
  {
    GameState::innerCrowdingRadius = value;
  }
  if (LoadConfigParameter(_config, "gamestate_outercrowdingradius", value))
  {
    GameState::outerCrowdingRadius = value;
  }
  if (LoadConfigParameter(_config, "gamestate_immobilitytimelimit", value))
  {
    GameState::immobilityTimeLimit = value;
  }
  if (LoadConfigParameter(_config, "gamestate_fallentimelimit", value))
  {
    GameState::fallenTimeLimit = value;
  }
  if (LoadConfigParameter(_config, "percept_hfov", value))
  {
    GameState::HFov = value;
  }
  if (LoadConfigParameter(_config, "percept_vfov", value))
  {
    GameState::VFov = value;
  }
  if (LoadConfigParameterBool(_config, "percept_restrictvision", boolValue))
  {
    GameState::restrictVision = boolValue;
  }
  if (LoadConfigParameterBool(_config, "percept_groundtruthinfo", boolValue))
  {
    GameState::groundTruthInfo = boolValue;
  }
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
    this->gameTime = this->cycleCounter * GameState::kCounterCycleTime;
  }
  this->lastCycleTimeLength = this->gameTime - this->prevCycleGameTime;

  this->hasCurrentStateChanged = false;
  if (this->currentState)
  {
    this->currentState->Update();
  }

  for (auto &team : this->teams)
  {
    for (auto &agent : team->members)
    {
      agent.prevPos = agent.pos;
    }
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
      agent.prevStatus = agent.status;
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
      agent.prevStatus = agent.status;
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
    if (this->currentState)
    {
      this->currentState->Uninitialize();
    }
    _newState->prevState = this->currentState;
    this->currentState = _newState;
    this->currentState->Preinitialize();
    this->hasCurrentStateChanged = true;
    gzmsg << "(" << this->gameTime <<
          ") playmode changed to " << this->currentState->name << std::endl;
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
        if (agent.pos.Distance(this->ballPos) < GameState::dropBallRadius)
        {
          if (fabs(agent.pos.X() - this->ballPos.X()) < GameState::kOnBallDist
              && fabs(agent.pos.Y() - this->ballPos.Y()) <
              GameState::kOnBallDist)
          {
            agent.pos.Set(agent.pos.X() + 2*GameState::kOnBallDist,
                          agent.pos.Y() + 2*GameState::kOnBallDist,
                          agent.pos.Z());
          }
          math::Line3<double> line(agent.pos, this->ballPos);
          math::Vector3<double> newPos;
          newPos.Set(0, 0, agent.bodyType->TorsoHeight() +
                     GameState::beamHeightOffset);
          math::Vector3<double> newPos2;
          newPos2.Set(0, 0, agent.bodyType->TorsoHeight() +
                      GameState::beamHeightOffset);
          if (Geometry::IntersectionCircumferenceLine(line, this->ballPos,
              GameState::kDropBallRadiusMargin +
              GameState::dropBallRadius, newPos, newPos2))
          {
            auto closerPos = newPos2;
            if (agent.pos.Distance(newPos) < agent.pos.Distance(newPos2))
            {
              closerPos = newPos;
            }
            this->MoveAgent(agent, closerPos);
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
    gzmsg << "first half over, switching to second half" << std::endl;
    this->SetHalf(Half::SECOND_HALF);
    this->SetCurrent(beforeKickOffState);
  }
  else if ((this->half == Half::SECOND_HALF)
           && (elapsedGameTime >= SecondsEachHalf))
  {
    // End of the game
    this->SetCurrent(gameOverState);
  }
}

/////////////////////////////////////////////////
bool GameState::IsBallInGoal(Team::Side _side)
{
  math::Box goalBox;
  math::Plane<double> goalPlane;
  if (_side == Team::Side::LEFT)
  {
    goalBox = SoccerField::kGoalBoxLeft;
    goalPlane = SoccerField::kGoalPlaneLeft;
  }
  else
  {
    goalBox = SoccerField::kGoalBoxRight;
    goalPlane = SoccerField::kGoalPlaneRight;
  }

  if (goalBox.Contains(this->ballPos))
  {
    return true;
  }

  double t;
  math::Vector3<double> pt;
  math::Line3<double> ballLine(
    this->ballPos - (this->GetElapsedCycleGameTime() * this->ballVel),
    this->ballPos);
  bool intersect = Geometry::IntersectionPlaneLine(ballLine, goalPlane, t, pt);
  if (intersect && t > 0 && t < 1 && fabs(pt.Y()) < SoccerField::kHalfGoalWidth
      && pt.Z() > 0 && pt.Z() < SoccerField::kGoalHeight)
  {
    return intersect;
  }

  return false;
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
  if (this->IsBallInGoal(Team::Side::LEFT))
  {
    this->SetCurrent(goalRightState);
  }
  else if (this->IsBallInGoal(Team::Side::RIGHT))
  {
    // The ball is inside the right goal.
    this->SetCurrent(goalLeftState);
  }
  else if (fabs(this->ballPos.Y()) > SoccerField::kHalfFieldHeight +
           SoccerField::kOutofBoundsTol)
  {
    // The ball is outside of the sideline.
    // Choose team
    if (lastContactSide == Team::Side::LEFT)
    {
      this->SetCurrent(kickInRightState);
    }
    else
    {
      this->SetCurrent(kickInLeftState);
    }
  }
  else if (fabs(this->ballPos.X()) > SoccerField::kHalfFieldWidth +
           SoccerField::kOutofBoundsTol)
  {
    // The ball is outside of the field
    // over the defensive team's goal line.
    if (this->ballPos.X() < 0)
    {
      // Choose team 1
      if (lastContactSide == Team::Side::LEFT)
      {
        this->SetCurrent(cornerKickRightState);
      }
      else
      {
        this->SetCurrent(goalKickLeftState);
      }
    }
    else
    {
      // Choose team 2
      if (lastContactSide == Team::Side::LEFT)
      {
        this->SetCurrent(goalKickRightState);
      }
      else
      {
        this->SetCurrent(cornerKickLeftState);
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckGoalKickIllegalDefense(const Team::Side _teamAllowed)
{
  math::Box penaltyBox;
  if (_teamAllowed == Team::Side::LEFT)
  {
    penaltyBox = SoccerField::kPenaltyBoxLeft;
  }
  else
  {
    penaltyBox = SoccerField::kPenaltyBoxRight;
  }

  for (auto &team : this->teams)
  {
    if (team->side != _teamAllowed)
    {
      for (auto &agent : team->members)
      {
        if (penaltyBox.Contains(agent.pos))
        {
          gzmsg << "CheckGoalKickIllegalDefense() violation" << std::endl;
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
      penaltyBox = SoccerField::kPenaltyBoxLeft;
      goalCenter = SoccerField::kGoalCenterLeft;
    }
    else
    {
      penaltyBox = SoccerField::kPenaltyBoxRight;
      goalCenter = SoccerField::kGoalCenterRight;
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
            Agent *bestAgent = nullptr;
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
            if (bestAgent)
            {
              gzmsg << "CheckIllegalDefense() violation" << std::endl;
              this->MoveAgentToSide(*bestAgent);
            }
            agent.inPenaltyBox = true;
          }
          else
          {
            gzmsg << "CheckIllegalDefense() violation" << std::endl;
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
  for (auto const &team : this->teams)
  {
    for (auto const &agent : team->members)
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
            gzmsg << "CheckCrowding() violation" << std::endl;
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
            gzmsg << "CheckCrowding() violation" << std::endl;
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
        agent.timeImmobilized += this->GetElapsedCycleGameTime();
      }
      else
      {
        agent.timeImmobilized = 0;
      }

      if (agent.pos.Z() < agent.bodyType->TorsoHeight() * 0.5)
      {
        agent.timeFallen += this->GetElapsedCycleGameTime();
      }
      else
      {
        agent.timeFallen = 0;
      }
      // move agent to side of field if they have remained fallen
      // or timeout too long.
      const double kScale = 1.0 + (agent.uNum == 1);
      if (agent.timeImmobilized >= kScale * GameState::immobilityTimeLimit
          || agent.timeFallen >= kScale * GameState::fallenTimeLimit)
      {
        agent.timeImmobilized = 0;
        agent.timeFallen = 0;
        gzmsg << "CheckImmobility() violation" << std::endl;
        this->MoveAgentToSide(agent);
      }
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

  // check and make sure that the first contact after kick off
  // (or second overall contact) is not by the same agent who performed
  // the kick off
  std::shared_ptr<BallContact> firstContact = this->ballContactHistory.at(1);
  if (this->touchBallKickoff
      && this->currentState->prevState
      && (this->currentState->prevState->name == KickOffRight
          || this->currentState->prevState->name == KickOffLeft)
      && this->touchBallKickoff->side == firstContact->side
      && this->touchBallKickoff->uNum == firstContact->uNum)
  {
    gzmsg << "CheckDoubleTouch() violation" << std::endl;
    if (this->currentState->prevState->side == Team::Side::LEFT)
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
  std::shared_ptr<BallContact> lastBallContact = this->GetLastBallContact();
  if (!lastBallContact)
  {
    return;
  }
  for (auto &team : this->teams)
  {
    if ((!team->canScore)
        && (this->touchBallKickoff)
        && ((lastBallContact->side != team->side)
            || (lastBallContact->side == team->side
                && this->touchBallKickoff->uNum != lastBallContact->uNum
                && lastBallContact->contactPos.Distance(
                  SoccerField::kCenterOfField) >
                SoccerField::kCenterCircleRadius)))
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
      {
        isOffSide = agent.pos.X() > 0;
      }
      else
      {
        isOffSide = agent.pos.X() < 0;
      }

      // if on kicking team, must stay in circle or cannot cross line.
      if (team->side == _kickingSide && (isOffSide &&
                                         agentPosNoZ.Distance(
                                           SoccerField::kCenterOfField) >
                                         SoccerField::kCenterCircleRadius))
      {
        // move them to side of field for now
        gzmsg << "CheckOffSidesOnKickOff() violation" << std::endl;
        this->MoveOffSideAgent(agent);

        // if on defending team, cannot cross line and go inside circle.
      }
      else if (team->side != _kickingSide &&
               (isOffSide || agentPosNoZ.Distance(SoccerField::kCenterOfField)
                < SoccerField::kCenterCircleRadius))
      {
        // move them to side of field for now
        gzmsg << "CheckOffSidesOnKickOff() violation" << std::endl;
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
  _agent.pos.Set(_x, _y, _agent.bodyType->TorsoHeight() +
                 GameState::beamHeightOffset);
  _agent.rot.Euler(0, 0, _yaw);
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                               const double _yaw, const bool _beamOnce) const
{
  double offsetX = (static_cast<double>(random()) / (RAND_MAX)) *
                   (2 * GameState::kBeamNoise) - GameState::kBeamNoise;
  double offsetY = (static_cast<double>(random()) / (RAND_MAX)) *
                   (2 * GameState::kBeamNoise) - GameState::kBeamNoise;
  double offsetYaw = (static_cast<double>(random()) / (RAND_MAX)) *
                     (2 * GameState::kBeamNoise) - GameState::kBeamNoise;
  double newX = _x + offsetX;
  double newY = _y + offsetY;
  double newYaw = _yaw + offsetYaw;

  if (_beamOnce
      && abs(_agent.pos.X() - newX) <= GameState::kBeamNoise
      && abs(_agent.pos.Y() - newY) <= GameState::kBeamNoise
      && abs(_agent.rot.Euler().Z() - newYaw) < GameState::kBeamNoise)
  {
    return;
  }

  _agent.pos.Set(_x + offsetX, _y + offsetY,
                 _agent.bodyType->TorsoHeight() +
                 GameState::beamHeightOffset);
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
    newY = -SoccerField::kHalfFieldHeight;
  }
  else
  {
    newY = SoccerField::kHalfFieldHeight;
  }
  _agent.pos = math::Vector3<double>(_agent.pos.X(), newY,
                                     _agent.bodyType->TorsoHeight() +
                                     GameState::beamHeightOffset);
  _agent.updatePose = true;
}

void GameState::MoveOffSideAgent(Agent &_agent) const
{
  if (_agent.team->side == Team::Side::LEFT)
  {
    _agent.pos.Set(-(SoccerField::kCenterCircleRadius + 0.5), _agent.pos.Y(),
                   _agent.bodyType->TorsoHeight() +
                   GameState::beamHeightOffset);
  }
  else if (_agent.team->side == Team::Side::RIGHT)
  {
    _agent.pos.Set(SoccerField::kCenterCircleRadius + 0.5, _agent.pos.Y(),
                   _agent.bodyType->TorsoHeight() +
                   GameState::beamHeightOffset);
  }
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
math::Vector3<double> GameState::GetBall() const
{
  return this->ballPos;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCenter()
{
  this->ballPos = SoccerField::kBallCenterPosition;
  this->ballVel = math::Vector3<double>(0, 0, 0);
  this->ballAngVel = math::Vector3<double>(0, 0, 0);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallForGoalKick(const Team::Side _side)
{
  double newX = SoccerField::kHalfFieldWidth - 1.0;
  if (_side == Team::Side::LEFT)
  {
    newX = -newX;
  }
  this->ballPos = math::Vector3<double>(newX, 0, SoccerField::kBallRadius);
  this->ballVel = math::Vector3<double>(0, 0, 0);
  this->ballAngVel = math::Vector3<double>(0, 0, 0);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCorner()
{
  this->ballPos = math::Vector3<double>(
                    (fabs(ballPos.X()) / ballPos.X()) *
                    SoccerField::kHalfFieldWidth,
                    (fabs(ballPos.Y()) / ballPos.Y()) *
                    SoccerField::kHalfFieldHeight,
                    SoccerField::kBallRadius);
  this->ballVel = math::Vector3<double>(0, 0, 0);
  this->ballAngVel = math::Vector3<double>(0, 0, 0);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallInBounds()
{
  double newX = std::max(std::min(SoccerField::kHalfFieldWidth, ballPos.X()),
                         -SoccerField::kHalfFieldWidth);
  double newY = std::max(std::min(SoccerField::kHalfFieldHeight, ballPos.Y()),
                         -SoccerField::kHalfFieldHeight);
  this->ballPos.Set(newX, newY, SoccerField::kBallRadius);
  this->ballVel = math::Vector3<double>(0, 0, 0);
  this->ballAngVel = math::Vector3<double>(0, 0, 0);
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBall(const math::Vector3<double> &_ballPos)
{
  this->ballPos = _ballPos;
  this->ballVel = math::Vector3<double>(0, 0, 0);
  this->ballAngVel = math::Vector3<double>(0, 0, 0);
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

/////////////////////////////////////////////////
ignition::math::Vector3<double> GameState::GetBallVel() const
{
  return this->ballVel;
}

/////////////////////////////////////////////////
ignition::math::Vector3<double> GameState::GetBallAngVel() const
{
  return this->ballAngVel;
}

////////////////////////////////////////////////
void GameState::Initialize()
{
  // do something when play mode changes
  // for now, do nothing
}

////////////////////////////////////////////////
Agent *GameState::AddAgent(const int _uNum, const std::string &_teamName,
                           const std::shared_ptr<NaoBT> &_bodyType,
                           const int _socketID)
{
  if (this->currentState->name != "BeforeKickOff")
  {
    gzmsg << "GameState::AddAgent() error: Invalid playmode, "
      << this->currentState->name << std::endl;
    return nullptr;
  }

  int uNum = _uNum;

  if (uNum < 0 || uNum > GameState::playerLimit)
  {
    gzmsg << "GameState::AddAgent() error: Invalid uNum, " << uNum <<
      std::endl;
    return nullptr;
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
      if (team->name == GameState::kDefaultTeamName
          && team->members.size() == 0)
      {
        teamToAdd = team;
        team->name = _teamName;
        break;
      }
    }
  }
  if (!teamToAdd)
  {
    gzmsg << "GameState::AddAgent() error: Invalid team, " << _teamName <<
      std::endl;
    return nullptr;
  }

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
      gzmsg << "GameState::AddAgent() error: uNum already in use, "
        << uNum << std::endl;
      return nullptr;
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
    gzmsg << "GameState::AddAgent() error: No available uNum to assign"
      << std::endl;
    return nullptr;
  }
  teamToAdd->members.push_back(Agent(uNum, teamToAdd, _bodyType, _socketID));
  return &teamToAdd->members.back();
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
  return false;
}

////////////////////////////////////////////////
bool GameState::BeamAgent(const int _uNum, const std::string &_teamName,
                          const double _x, const double _y, const double _rot)
{
  if (this->currentState->name != BeforeKickOff
      && this->currentState->name != GoalKickLeft
      && this->currentState->name != GoalKickRight)
  {
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
          if (team->side == Team::Side::LEFT)
          {
            this->MoveAgentNoisy(agent, _x, _y, IGN_DTOR(_rot));
          }
          else
          {
            double rot = _rot + 180.0;
            if (rot > 360.0)
            {
              rot -= 360.0;
            }
            this->MoveAgentNoisy(agent, -_x, -_y, IGN_DTOR(rot));
          }
          return true;
        }
      }
    }
  }
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
double GameState::GetElapsedGameTime(const bool _beginning) const
{
  if (_beginning && this->GetHalf() == Half::SECOND_HALF)
  {
    return (this->gameTime - this->startGameTime) +
           GameState::SecondsEachHalf;
  }
  else
  {
    return this->gameTime - this->startGameTime;
  }
}

////////////////////////////////////////////////
double GameState::GetElapsedCycleGameTime() const
{
  return this->lastCycleTimeLength;
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
    double newGameTime = GameState::kCounterCycleTime * cycleCounter;
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
Team::Side GameState::GetLastSideTouchedBall() const
{
  if (this->GetLastBallContact())
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
    return nullptr;
  }
}
