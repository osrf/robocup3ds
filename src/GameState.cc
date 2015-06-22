/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may !use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law || agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES || CONDITIONS OF ANY KIND, either express || implied.
 * See the License for the specific language governing permissions &&
 * limitations under the License.
 *
*/
#include <boost/bind.hpp>
#include <sdf/sdf.hh>
#include <iostream>
#include <vector>
#include <algorithm>
#include <string>
#include <map>

#include "robocup3ds/GameState.hh"
#include "robocup3ds/SoccerField.hh"
#include "robocup3ds/Geometry.hh"

using namespace ignition;

const std::string GameState::BeforeKickOff   = "BeforeKickOff";
const std::string GameState::KickOffLeft     = "KickOff_Left";
const std::string GameState::KickOffRight    = "KickOff_Right";
const std::string GameState::Play            = "PlayOn";
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
const std::string GameState::FreeKickRight   = "kick_kick_right";

std::map<std::string, std::string> *config = NULL;

// default values for configuration variables
double GameState::SecondsFullGame = 600;
double GameState::SecondsEachHalf = SecondsFullGame * 0.5;
double GameState::SecondsGoalPause = 3;
double GameState::SecondsKickInPause = 1;
double GameState::SecondsKickIn = 15;
double GameState::SecondsBeforeKickOff = 5;
double GameState::SecondsKickOff = 15;
bool GameState::useCounterForGameTime = true;
int GameState::playerLimit = 11;
int GameState::penaltyBoxLimit = 3;
double GameState::beamHeight = SoccerField::NaoPoseHeight + 0.05;
double GameState::crowdingEnableRadius = 0.8;
double GameState::innerCrowdingRadius = 0.4;
double GameState::outerCrowdingRadius = 1.0;
double GameState::immobilityTimeLimit = 15;
double GameState::fallenTimeLimit = 30;
double GameState::dropBallRadius = 2;

/////////////////////////////////////////////////
GameState::GameState():
  beforeKickOffState(new BeforeKickOffState(BeforeKickOff, this)),
  kickOffLeftState(new KickOffLeftState(KickOffLeft, this)),
  kickOffRightState(new KickOffRightState(KickOffRight, this)),
  playState(new PlayState(Play, this)),
  kickInLeftState(new KickInLeftState(KickInLeft, this)),
  kickInRightState(new KickInRightState(KickInRight, this)),
  cornerKickLeftState(new CornerKickLeftState(CornerKickLeft, this)),
  cornerKickRightState(new CornerKickRightState(CornerKickRight, this)),
  goalKickLeftState(new GoalKickLeftState(GoalKickLeft, this)),
  goalKickRightState(new GoalKickRightState(GoalKickRight, this)),
  gameOverState(new GameOverState(GameOver, this)),
  goalLeftState(new GoalLeftState(GoalLeft, this)),
  goalRightState(new GoalRightState(GoalRight, this)),
  freeKickLeftState(new FreeKickLeftState(FreeKickLeft, this)),
  freeKickRightState(new FreeKickRightState(FreeKickRight, this))
{
  this->loadConfiguration();
  this->half = FIRST_HALF;
  this->cycleCounter = 0;
  this->gameTime = this->prevCycleGameTime = this->startGameTime = 0.0;
  this->updateBallPose = false;
  this->currentState = NULL;
  this->touchBallKickoff = NULL;
  this->hasCurrentStateChanged = false;
  this->SetCurrent(beforeKickOffState.get());
  this->teams.push_back(new Team("_unnamed_team", Team::LEFT, 0,
                                 GameState::playerLimit));
  this->teams.push_back(new Team("_unnamed_team", Team::RIGHT, 0,
                                 GameState::playerLimit));
}

/////////////////////////////////////////////////
GameState::~GameState()
{
  // delete teams when shutting down game
  for (size_t i = 0; i < this->teams.size(); i++)
  {
    delete this->teams.at(i);
  }
  // clear list of ball contacts
  clearBallContactHistory();
}

/////////////////////////////////////////////////
void GameState::loadConfiguration()
{
  if (config == NULL)
  {
    return;
  }
  double value;
  bool boolValue;
  if (loadConfigParameter("gamestate_secondsfullgame", value))
  {
    GameState::SecondsFullGame = value;
    GameState::SecondsEachHalf = 0.5 * GameState::SecondsFullGame;
  }
  else if (loadConfigParameter("gamestate_secondseachhalf", value))
  {
    GameState::SecondsEachHalf = value;
    GameState::SecondsFullGame = 2.0 * GameState::SecondsEachHalf;
  }
  if (loadConfigParameter("gamestate_secondsgoalpause", value))
    GameState::SecondsGoalPause = value;
  if (loadConfigParameter("gamestate_secondskickinpause", value))
    GameState::SecondsKickInPause = value;
  if (loadConfigParameter("gamestate_secondskickin", value))
    GameState::SecondsKickIn = value;
  if (loadConfigParameter("gamestate_secondsbeforekickoff", value))
    GameState::SecondsBeforeKickOff = value;
  if (loadConfigParameter("gamestate_secondskickoff", value))
    GameState::SecondsKickOff = value;
  if (loadConfigParameter("gamestate_dropballradius", value))
    GameState::dropBallRadius = value;
  if (loadConfigParameterBool("gamestate_usecounterforgametime", boolValue))
    GameState::useCounterForGameTime = boolValue;
  if (loadConfigParameter("gamestate_playerlimit", value))
    GameState::playerLimit = static_cast<int>(value);
  if (loadConfigParameter("gamestate_penaltyboxlimit", value))
    GameState::penaltyBoxLimit = static_cast<int>(value);
  if (loadConfigParameter("gamestate_beamheight", value))
    GameState::beamHeight = value;
  if (loadConfigParameter("gamestate_crowdingenableradius", value))
    GameState::crowdingEnableRadius = value;
  if (loadConfigParameter("gamestate_innercrowdingradius", value))
    GameState::innerCrowdingRadius = value;
  if (loadConfigParameter("gamestate_outercrowdingradius", value))
    GameState::outerCrowdingRadius = value;
  if (loadConfigParameter("gamestate_immobilitytimelimit", value))
    GameState::immobilityTimeLimit = value;
  if (loadConfigParameter("gamestate_fallentimelimit", value))
    GameState::fallenTimeLimit = value;
}

/////////////////////////////////////////////////
bool GameState::loadConfigParameter(const std::string &_key,
                                    double &_value)
{
  try
  {
    _value = std::stod((*config)[_key]);
  }
  catch (...)
  {
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
bool GameState::loadConfigParameterBool(const std::string &_key,
                                    bool &_boolValue)
{
  try
  {
    _boolValue = (*config)[_key] != "false";
  }
  catch (...)
  {
    return false;
  }
  return true;
}

/////////////////////////////////////////////////
void GameState::clearBallContactHistory()
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
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    for (size_t j = 0; j < this->teams.at(i)->members.size(); ++j)
    {
      Agent &agent = this->teams.at(i)->members.at(j);
      agent.status = Agent::RELEASED;
    }
  }
}

/////////////////////////////////////////////////
void GameState::StopPlayers()
{
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    for (size_t j = 0; j < this->teams.at(i)->members.size(); ++j)
    {
      Agent &agent = this->teams.at(i)->members.at(j);
      agent.status = Agent::STOPPED;
    }
  }
}

////////////////////////////////////////////////
void GameState::SetCurrent(State *_newState)
{
  // Only update the state if _newState is different than the current state.
  if (this->currentState != _newState)
  {
    this->Initialize();
    if (this->currentState != NULL)
    {
      this->currentState->unInitialize();
    }
    _newState->prevState = this->currentState;
    this->currentState = _newState;
    this->currentState->preInitialize();
    this->hasCurrentStateChanged = true;
  }
}

/////////////////////////////////////////////////
void GameState::DropBallImpl(const Team::Side _teamAllowed)
{
  // Check if the player is withing FREE_KICK distance.
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    Team *team = this->teams.at(i);

    if (team->side != _teamAllowed)
    {
      for (size_t j = 0; j < team->members.size(); ++j)
      {
        Agent &agent = team->members.at(j);
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

  double elapsedGameTime = this->getElapsedGameTime();
  if ((this->half == FIRST_HALF) && (elapsedGameTime >= SecondsEachHalf))
  {
    // swap team sides
    Team::Side temp = this->teams.at(0)->side;
    this->teams.at(0)->side = this->teams.at(1)->side;
    this->teams.at(1)->side = temp;

    // End of the first half
    this->startGameTime = this->gameTime;
    this->SetHalf(SECOND_HALF);
    this->SetCurrent(kickOffRightState.get());
  }
  else if ((this->half == SECOND_HALF) && (elapsedGameTime >= SecondsEachHalf))
  {
    // End of the game
    this->SetCurrent(gameOverState.get());
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
    lastContactSide = this->getLastBallContact()->side;
  }
  else
  {
    if (this->half == FIRST_HALF)
    {
      lastContactSide = Team::LEFT;
    }
    else
    {
      lastContactSide = Team::RIGHT;
    }
  }

  // The ball is inside the left goal.
  if (SoccerField::GoalBoxLeft.Contains(this->ballPos))
  {
    this->SetCurrent(goalRightState.get());
  }
  else if (SoccerField::GoalBoxRight.Contains(this->ballPos))
  {
    // The ball is inside the right goal.
    this->SetCurrent(goalLeftState.get());
  }
  else if (fabs(this->ballPos.Y()) > SoccerField::HalfFieldHeight +
           SoccerField::OutofBoundsTol)
  {
    // The ball is outside of the sideline.
    // Choose team
    if (lastContactSide == Team::LEFT)
    {
      this->SetCurrent(kickInRightState.get());
    }
    else
    {
      this->SetCurrent(kickInLeftState.get());
    }
  }
  else if (fabs(this->ballPos.X()) > SoccerField::HalfFieldWidth +
           SoccerField::OutofBoundsTol)
  {
    // The ball is outside of the field over the defensive team's goal line.
    if (this->ballPos.X() < 0)
    {
      // Choose team 1
      if (lastContactSide == Team::LEFT)
      {
        this->SetCurrent(cornerKickRightState.get());
      }
      else
      {
        this->SetCurrent(goalKickLeftState.get());
      }
    }
    else
    {
      // Choose team 2
      if (lastContactSide == Team::LEFT)
      {
        this->SetCurrent(goalKickRightState.get());
      }
      else
      {
        this->SetCurrent(cornerKickLeftState.get());
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckGoalKickIllegalDefense(Team::Side _teamAllowed)
{
  math::Box penaltyBox;
  if (_teamAllowed == Team::LEFT)
  {
    penaltyBox = SoccerField::PenaltyBoxLeft;
  }
  else
  {
    penaltyBox = SoccerField::PenaltyBoxRight;
  }

  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    Team *team = this->teams.at(i);
    if (team->side != _teamAllowed)
    {
      for (size_t j = 0; j < team->members.size(); j++)
      {
        Agent &agent = team->members.at(j);
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
  for (size_t k = 0; k < this->teams.size(); ++k)
  {
    Team *team = this->teams.at(k);
    if (team->side == Team::LEFT)
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
    for (size_t i = 0; i < team->members.size(); i++)
    {
      Agent &agent = team->members.at(i);
      if (!penaltyBox.Contains(agent.pos) && agent.inPenaltyBox)
      {
        agent.inPenaltyBox = false;
        team->numPlayersInPenaltyBox--;
      }
    }

    // do bookkeeping for agents that enter penalty box
    for (size_t i = 0; i < team->members.size(); i++)
    {
      Agent &agent = team->members.at(i);
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
          // if agent is !goalie: move agent away if penalty box is
          // already crowded
          // if agent is goalie: move another agent that is farthest
          // from goal away
          if (agent.isGoalKeeper())
          {
            double bestDist = -1;
            Agent *bestAgent = NULL;
            for (size_t j = 0; j < team->members.size(); j++)
            {
              Agent &nonGoalieAgent = team->members.at(j);
              if (nonGoalieAgent.isGoalKeeper())
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
          // agent is !penalty box nor is accounted for so do nothing
        }
      }
    }
  }
}

/// \Brief Struct for helping to sort agents by their distances,
/// used by CheckCrowding_helper only
struct AgentDist
{
  GameState::Agent *agent;
  double dist;
};
/// \Brief Sorting function for AgentDist
bool sortDist (AgentDist _i, AgentDist _j) { return (_i.dist < _j.dist); }

/////////////////////////////////////////////////
void GameState::CheckCrowding()
{
  bool enableCrowding = false;
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    Team *team = this->teams.at(i);
    for (size_t j = 0; j < team->members.size(); j++)
    {
      Agent &agent = team->members.at(j);
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
    for (size_t k = 0; k < this->teams.size(); ++k)
    {
      Team *team = this->teams.at(k);
      std::vector<AgentDist> agentDists;
      for (size_t i = 0; i < team->members.size(); i++)
      {
        AgentDist agentDist;
        agentDist.agent = &team->members.at(i);
        agentDist.dist = agentDist.agent->pos.Distance(ballPos);
        agentDists.push_back(agentDist);
      }
      std::sort(agentDists.begin(), agentDists.end(), sortDist);

      // only allow one agent to be in inner radius
      int reposition_2 = 1;
      for (size_t i = 0; i < agentDists.size(); i++)
      {
        AgentDist agentDist = agentDists.at(i);
        Agent *agent = agentDist.agent;
        if (agentDist.dist < GameState::innerCrowdingRadius)
        {
          if (reposition_2 > 0)
          {
            reposition_2--;
          }
          else
          {
            this->MoveAgentToSide(*agent);
          }
        }
      }
      // only allow two agents to be in outer radius
      int reposition_3 = 2;
      for (size_t i = 0; i < agentDists.size(); i++)
      {
        AgentDist agentDist = agentDists.at(i);
        Agent *agent = agentDist.agent;
        if (agentDist.dist < GameState::outerCrowdingRadius)
        {
          if (reposition_3 > 0)
          {
            reposition_3--;
          }
          else
          {
            this->MoveAgentToSide(*agent);
          }
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckImmobility()
{
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    Team *team = this->teams.at(i);
    for (size_t j = 0; j < team->members.size(); ++j)
    {
      Agent &agent = team->members.at(j);

      if (agent.pos == agent.prevPos)
      {
        agent.timeImmoblized += this->getElapsedCycleGameTime();
      }
      else
      {
        agent.timeImmoblized = 0;
      }

      if (agent.pos.Z() < SoccerField::NaoPoseHeight * 0.5)
      {
        agent.timeFallen += this->getElapsedCycleGameTime();
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
  // (or second overall contact) is !by the same agent who performed
  // the kick off
  BallContact *firstContact = this->ballContactHistory.at(1).get();
  if (this->touchBallKickoff != NULL
      && this->currentState->prevState != NULL
      && (this->currentState->prevState->name == "KickOff_Left"
          || this->currentState->prevState->name == "KickOff_Right")
      && this->touchBallKickoff->side == firstContact->side
      && this->touchBallKickoff->uNum == firstContact->uNum)
  {
    if (this->currentState->prevState->name == "KickOff_Left")
    {
      this->SetCurrent(kickOffRightState.get());
    }
    else
    {
      this->SetCurrent(kickOffLeftState.get());
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckCanScore()
{
  BallContact *ballContact = this->getLastBallContact();
  if (ballContact == NULL)
  {
    return;
  }
  for (size_t j = 0; j < this->teams.size(); j++)
  {
    Team *team = this->teams.at(j);
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

void GameState::CheckOffSidesOnKickOff(Team::Side _kickingSide)
{
  // check for agents that violate sides
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    Team *team = this->teams.at(i);
    for (size_t j = 0; j < team->members.size(); ++j)
    {
      Agent &agent = team->members.at(j);
      math::Vector3<double> agentPosNoZ(agent.pos.X(), agent.pos.Y(), 0);

      bool isOffSide;
      if (team->side == Team::LEFT)
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
                                           SoccerField::CenterOfField)
                                         > SoccerField::CenterCircleRadius))
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
                          const math::Vector3<double> &_pos)
{
  _agent.pos = _pos;
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgent(Agent &_agent, const double _x, const double _y,
                          const double _yaw)
{
  _agent.pos.Set(_x, _y, GameState::beamHeight);
  _agent.rot.Euler(0, 0, _yaw);
  _agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                               const double _yaw)
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
                          const math::Quaternion<double> &_rot)
{
  _agent.pos = _pos;
  _agent.rot = _rot;
  _agent.updatePose = true;
}

void GameState::MoveAgentToSide(Agent &_agent)
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

void GameState::MoveOffSideAgent(Agent &_agent)
{
  if (_agent.team->side == Team::LEFT)
  {
    _agent.pos.Set(-(SoccerField::CenterCircleRadius + 0.5), _agent.pos.Y(),
                   GameState::beamHeight);
  }
  else if (_agent.team->side == Team::RIGHT)
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
void GameState::setBallVel(const math::Vector3<double> &_ballVel)
{
  this->ballVel = _ballVel;
  this->updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::setBallAngVel(const math::Vector3<double> &_ballAngVel)
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
bool GameState::addAgent(int _uNum, std::string _teamName)
{
  // std::cout << "adding agent: " << uNum << " teamName: "
  // << teamName << std::endl;
  if (_uNum < 0 || _uNum > GameState::playerLimit)
  {
    // std::cout << "uNum " << uNum << " is invalid, cannot add agent to team "
    // << teamName << "!" << std::endl;
    return false;
  }

  int teamToAdd = -1;
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    if (this->teams.at(i)->name == _teamName)
    {
      teamToAdd = i;
    }
  }
  if (teamToAdd == -1)
  {
    for (size_t i = 0; i < this->teams.size(); ++i)
    {
      if (this->teams.at(i)->name == "_unnamed_team"
          && this->teams.at(i)->members.size() == 0)
      {
        teamToAdd = i;
        this->teams.at(i)->name = _teamName;
        break;
      }
    }
  }
  if (teamToAdd == -1)
  {
    // std::cout << uNum << " " << teamName.c_str() << std::endl;
    // std::cout << "There already are two teams, cannot add agent into
    // new team!" << std::endl;
    return false;
  }

  Team *team = this->teams.at(teamToAdd);
  if (static_cast<int>(team->members.size()) > GameState::playerLimit)
  {
    // std::cout << "Team is already full, cannot add agent
    // to team!" << std::endl;
    return false;
  }
  bool *uNumArray = new bool[GameState::playerLimit];
  for (int i = 0; i < GameState::playerLimit; i++)
  {
    uNumArray[i] = true;
  }
  for (size_t i = 0; i < team->members.size(); i++)
  {
    uNumArray[team->members.at(i).uNum - 1] = false;
    if (_uNum != 0 && team->members.at(i).uNum == _uNum)
    {
      // std::cout << "Already have an agent with this unum: " << uNum <<
      // ", cannot add agent to team!" << std::endl;
      delete[] uNumArray;
      return false;
    }
  }
  if (_uNum == 0)
  {
    for (int i = 0; i < GameState::playerLimit; i++)
    {
      if (uNumArray[i])
      {
        _uNum = i + 1;
        break;
      }
    }
  }
  if (_uNum == 0)
  {
    // std::cout << "No free uNums avaliable, cannot add agent to team!"
    // << std::endl;
    delete[] uNumArray;
    return false;
  }
  team->members.push_back(Agent(_uNum, team));
  delete[] uNumArray;
  return true;
}


///////////////////////////////////////////////
bool GameState::removeAgent(int _uNum, std::string _teamName)
{
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    if (this->teams.at(i)->name == _teamName)
    {
      for (size_t j = 0; j < this->teams.at(i)->members.size(); j++)
      {
        if (this->teams.at(i)->members.at(j).uNum == _uNum)
        {
          this->teams.at(i)->members.erase(
            this->teams.at(i)->members.begin() + j);
          return true;
        }
      }
    }
  }
  // std::cout << "Agent !found, unable to remove agent!" << std::endl;
  return false;
}

////////////////////////////////////////////////
bool GameState::beamAgent(int _uNum, std::string _teamName,
                          double _x, double _y, double _rot)
{
  if (this->currentState->name != "BeforeKickOff"
      && this->currentState->name != "goal_kick_left"
      && this->currentState->name != "goal_kick_right")
  {
    // std::cout << "Incorrect play mode, unable to beam agent!" << std::endl;
    return false;
  }
  for (size_t i = 0; i < this->teams.size(); ++i)
  {
    if (this->teams.at(i)->name == _teamName)
    {
      for (size_t j = 0; j < this->teams.at(i)->members.size(); j++)
      {
        if (this->teams.at(i)->members.at(j).uNum == _uNum)
        {
          this->MoveAgentNoisy(this->teams.at(i)->members.at(j), _x, _y, _rot);
          return true;
        }
      }
    }
  }
  // std::cout << "Agent !found, unable to beam agent!" << std::endl;
  return false;
}
