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
#include <iostream>
#include <boost/bind.hpp>
#include <sdf/sdf.hh>

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

const double GameState::SecondsFullGame = 600;
const double GameState::SecondsEachHalf = SecondsFullGame * 0.5;
const double GameState::SecondsGoalPause = 3;
const double GameState::SecondsKickInPause = 1;
const double GameState::SecondsKickIn = 15;
const double GameState::SecondsBeforeKickOff = 5;
const double GameState::SecondsKickOff = 15;
const bool GameState::useCounterForGameTime = true;
const int GameState::playerLimit = 11;
const int GameState::penaltyBoxLimit = 3;
const double GameState::beamHeight = SoccerField::NaoPoseHeight + 0.05;
const double GameState::crowdingEnableDist = 0.8;
const double GameState::crowdingReposDist2 = 0.4;
const double GameState::crowdingReposDist3 = 1.0;
const double GameState::immobilityTimeLimit = 15;
const double GameState::fallenTimeLimit = 30;

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
  half = FIRST_HALF;
  cycleCounter = 0;
  gameTime = prevCycleGameTime = startGameTime = 0.0;
  updateBallPose = false;
  currentState = NULL;
  touchBallKickoff = NULL;
  hasCurrentStateChanged = false;
  SetCurrent(beforeKickOffState.get());
  teams.push_back(new Team("_unnamed_team", Team::LEFT, 0, GameState::playerLimit));
  teams.push_back(new Team("_unnamed_team", Team::RIGHT, 0, GameState::playerLimit));
}

/////////////////////////////////////////////////
GameState::~GameState()
{
  //delete teams when shutting down game
  for (size_t i = 0; i < teams.size(); i++) {
    delete teams.at(i);
  }
  //clear list of ball contacts
  clearBallContactHistory();
}

/////////////////////////////////////////////////
void GameState::clearBallContactHistory()
{
  ballContactHistory.clear();
}

/////////////////////////////////////////////////
void GameState::Update()
{
  cycleCounter++;
  if (GameState::useCounterForGameTime) {
    gameTime = cycleCounter * 0.02;
  }
  prevCycleGameTime = gameTime;

  hasCurrentStateChanged = false;
  if (currentState) {
    currentState->Update();
  }
}

/////////////////////////////////////////////////
void GameState::ReleasePlayers()
{
  for (size_t i = 0; i < teams.size(); ++i) {
    for (size_t j = 0; j < teams.at(i)->members.size(); ++j) {
      Agent &agent = teams.at(i)->members.at(j);
      agent.status = Agent::RELEASED;

    }
  }
}

/////////////////////////////////////////////////
void GameState::StopPlayers()
{
  for (size_t i = 0; i < teams.size(); ++i) {
    for (size_t j = 0; j < teams.at(i)->members.size(); ++j) {
      Agent &agent = teams.at(i)->members.at(j);
      agent.status = Agent::STOPPED;
    }
  }
}

////////////////////////////////////////////////
void GameState::SetCurrent(State *_newState)
{
  // Only update the state if _newState is different than the current state.
  if (currentState != _newState) {
    Initialize();
    if (currentState != NULL) {
      currentState->unInitialize();
    }
    _newState->prevState = currentState;
    currentState = _newState;
    currentState->preInitialize();
    hasCurrentStateChanged = true;
  }
}

/////////////////////////////////////////////////
void GameState::DropBallImpl(const Team::Side _teamAllowed)
{
  // Check if the player is withing FREE_KICK distance.
  for (size_t i = 0; i < teams.size(); ++i) {
    Team *team = teams.at(i);

    if (team->side != _teamAllowed) {
      for (size_t j = 0; j < team->members.size(); ++j) {
        Agent &agent = team->members.at(j);
        if (agent.pos.Distance(ballPos) < SoccerField::FreeKickMoveDist) {
          MoveAgentToSide(agent);
        }
        // math::Vector3<double> agentPos = agent.pos;
        // Move the player if it's close enough to the ball.
        // if (agentPos.Distance(ballPos) < SoccerField::FreeKickMoveDist) {

        //  // Calculate the general form equation of a line from two points.
        //  // a = y1 - y2
        //  // b = x2 - x1
        //  // c = (x1-x2)*y1 + (y2-y1)*x1
        //  math::Vector3<double> v(ballPos.Y() - agentPos.Y(),
        //                          agentPos.X() - ballPos.X(),
        //                          (ballPos.X() - agentPos.X()) * ballPos.Y() +
        //                          (agentPos.Y() - ballPos.Y()) * ballPos.X());
        //  math::Vector3<double> int1;
        //  int1.Set(0, 0, beamHeight);
        //  math::Vector3<double> int2;
        //  int2.Set(0, 0, beamHeight);
        //  if (Geometry::IntersectionCircunferenceLine(
        //        v, ballPos, SoccerField::FreeKickMoveDist, int1, int2)) {
        //    if (agentPos.Distance(int1) < agentPos.Distance(int2))
        //      MoveAgent(agent, int1);
        //    else
        //      MoveAgent(agent, int2);
        //  }
      }
    }
  }
}


/////////////////////////////////////////////////
void GameState::CheckTiming()
{
  if (hasCurrentStateChanged) {
    return;
  }

  double elapsedGameTime = getElapsedGameTime();
  if ((half == FIRST_HALF) && (elapsedGameTime >= SecondsEachHalf)) {
    //swap team sides
    Team::Side temp = teams.at(0)->side;
    teams.at(0)->side = teams.at(1)->side;
    teams.at(1)->side = temp;

    // End of the first half
    startGameTime = gameTime;
    SetHalf(SECOND_HALF);
    SetCurrent(kickOffRightState.get());
  } else if ((half == SECOND_HALF) && (elapsedGameTime >= SecondsEachHalf)) {
    // End of the game
    SetCurrent(gameOverState.get());
  }
}

/////////////////////////////////////////////////
void GameState::CheckBall()
{
  if (hasCurrentStateChanged) {
    return;
  }
  Team::Side lastContactSide;
  if (ballContactHistory.size() > 0) {
    lastContactSide = getLastBallContact()->side;
  } else {
    if (half == FIRST_HALF) {
      lastContactSide = Team::LEFT;
    } else {
      lastContactSide = Team::RIGHT;
    }
  }

  // The ball is inside the left goal.
  if (SoccerField::GoalBoxLeft.Contains(ballPos)) {
    SetCurrent(goalRightState.get());
  }
  // The ball is inside the right goal.
  else if (SoccerField::GoalBoxRight.Contains(ballPos)) {
    SetCurrent(goalLeftState.get());
  }
  // The ball is outside of the sideline.
  else if (fabs(ballPos.Y()) > SoccerField::HalfFieldHeight + SoccerField::OutofBoundsTol) {

    // Choose team
    if (lastContactSide == Team::LEFT)
      SetCurrent(kickInRightState.get());
    else
      SetCurrent(kickInLeftState.get());
  }
  // The ball is outside of the field over the defensive team's goal line.
  else if (fabs(ballPos.X()) > SoccerField::HalfFieldWidth + SoccerField::OutofBoundsTol) {
    if (ballPos.X() < 0) {
      // Choose team 1
      if (lastContactSide == Team::LEFT)
        SetCurrent(cornerKickRightState.get());
      else
        SetCurrent(goalKickLeftState.get());
    } else {
      // Choose team 2
      if (lastContactSide == Team::LEFT)
        SetCurrent(goalKickRightState.get());
      else
        SetCurrent(cornerKickLeftState.get());
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckGoalKickIllegalDefense(Team::Side _teamAllowed)
{
  math::Box penaltyBox;
  if (_teamAllowed == Team::LEFT) {
    penaltyBox = SoccerField::PenaltyBoxLeft;
  } else {
    penaltyBox = SoccerField::PenaltyBoxRight;
  }

  for (size_t i = 0; i < teams.size(); ++i) {
    Team *team = teams.at(i);
    if (team->side != _teamAllowed) {
      for (size_t j = 0; j < team->members.size(); j++) {
        Agent &agent = team->members.at(j);
        if (penaltyBox.Contains(agent.pos)) {
          MoveAgentToSide(agent);
        }
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckIllegalDefense()
{
  for (size_t i = 0; i < teams.size(); ++i) {
    CheckIllegalDefense_helper(teams.at(i));
  }
}

/////////////////////////////////////////////////
void GameState::CheckIllegalDefense_helper(Team *team)
{
  math::Box penaltyBox;
  math::Vector3<double> goalCenter;
  if (team->side == Team::LEFT) {
    penaltyBox = SoccerField::PenaltyBoxLeft;
    goalCenter = SoccerField::GoalCenterLeft;
  } else {
    penaltyBox = SoccerField::PenaltyBoxRight;
    goalCenter = SoccerField::GoalCenterRight;
  }

  //do bookkeeping for agents that leave penalty box
  for (size_t i = 0; i < team->members.size(); i++) {
    Agent &agent = team->members.at(i);
    if (not penaltyBox.Contains(agent.pos) and agent.inPenaltyBox) {
      agent.inPenaltyBox = false;
      team->numPlayersInPenaltyBox--;
    }
  }

  //do bookkeeping for agents that enter penalty box
  for (size_t i = 0; i < team->members.size(); i++) {
    Agent &agent = team->members.at(i);
    if (not agent.inPenaltyBox) {
      if (penaltyBox.Contains(agent.pos) and team->numPlayersInPenaltyBox < GameState::penaltyBoxLimit) {
        //account for agent if there is still room in penalty box
        team->numPlayersInPenaltyBox++;
        agent.inPenaltyBox = true;
      } else if (penaltyBox.Contains(agent.pos) and team->numPlayersInPenaltyBox >= GameState::penaltyBoxLimit)  {
        //if agent is not goalie: move agent away if penalty box is already crowded
        //if agent is goalie: move another agent away
        if (agent.isGoalKeeper()) {
          double bestDist = -1;
          Agent *bestAgent = NULL;
          for (size_t j = 0; j < team->members.size(); j++) {
            Agent &otherAgent = team->members.at(j);
            double otherAgentDist = agent.pos.Distance(goalCenter);
            if (agent.inPenaltyBox and agent.pos.Distance(goalCenter) > bestDist) {
              bestDist = otherAgentDist;
              bestAgent = &otherAgent;
            }
          }
          MoveAgentToSide(*bestAgent);
          agent.inPenaltyBox = true;
        } else {
          MoveAgentToSide(agent);
        }
      } else {
        //agent is not penalty box nor is accounted for so do nothing
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckCrowding()
{
  bool enableCrowding = false;
  for (size_t i = 0; i < teams.size(); ++i) {
    Team *team = teams.at(i);
    for (size_t j = 0; j < team->members.size(); j++) {
      Agent &agent = team->members.at(j);
      if (agent.pos.Distance(ballPos) < GameState::crowdingEnableDist) {
        enableCrowding = true;
        goto exitLoop;
      }
    }
  }
exitLoop:

  if (enableCrowding) {
    for (size_t i = 0; i < teams.size(); ++i) {
      CheckCrowding_helper(teams.at(i));
    }
  }
}

/// \Brief Struct for helping to sort agents by their distances, used by CheckCrowding_helper only
struct AgentDist {
  GameState::Agent *agent;
  double dist;
};
/// \Brief Sorting function for AgentDist
bool sortDist (AgentDist i, AgentDist j) { return (i.dist < j.dist); }

/////////////////////////////////////////////////
void GameState::CheckCrowding_helper(Team *team)
{
  //sort agents by their distances
  std::vector<AgentDist> agentDists;
  for (size_t i = 0; i < team->members.size(); i++) {
    AgentDist agentDist;
    agentDist.agent = &team->members.at(i);
    agentDist.dist = agentDist.agent->pos.Distance(ballPos);
    agentDists.push_back(agentDist);
  }
  std::sort(agentDists.begin(), agentDists.end(), sortDist);

  //only allow one agent to be in inner radius
  int reposition_2 = 1;
  for (size_t i = 0; i < agentDists.size(); i++) {
    AgentDist agentDist = agentDists.at(i);
    Agent *agent = agentDist.agent;
    if (agentDist.dist < GameState::crowdingReposDist2) {
      if (reposition_2 > 0) {
        reposition_2--;
      } else {
        MoveAgentToSide(*agent);
      }
    }
  }
  //only allow two agents to be in outer radius
  int reposition_3 = 2;
  for (size_t i = 0; i < agentDists.size(); i++) {
    AgentDist agentDist = agentDists.at(i);
    Agent *agent = agentDist.agent;
    if (agentDist.dist < GameState::crowdingReposDist3) {
      if (reposition_3 > 0) {
        reposition_3--;
      } else {
        MoveAgentToSide(*agent);
      }
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckImmobility()
{
  for (size_t i = 0; i < teams.size(); ++i) {
    Team *team = teams.at(i);
    for (size_t j = 0; j < team->members.size(); ++j) {
      Agent &agent = team->members.at(j);

      if (agent.pos == agent.prevPos) {
        agent.timeImmoblized += getElapsedCycleGameTime();
      } else {
        agent.timeImmoblized = 0;
      }

      if (agent.pos.Z() < SoccerField::NaoPoseHeight * 0.5) {
        agent.timeFallen += getElapsedCycleGameTime();
      } else {
        agent.timeFallen = 0;
      }

      //move agent to side of field if they have remained fallen or timeout too long.
      double SCALE = 1.0 + (agent.uNum == 1);
      if (agent.timeImmoblized >= SCALE * GameState::immobilityTimeLimit or agent.timeFallen >= SCALE * GameState::fallenTimeLimit) {
        agent.timeImmoblized = 0;
        agent.timeFallen = 0;
        MoveAgentToSide(agent);
      }
      agent.prevPos = agent.pos;
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckDoubleTouch()
{
  if (ballContactHistory.size() != 2 or hasCurrentStateChanged) {
    return;
  }

  //check and make sure that the first contact after kick off (or third overall contact) is not by the same agent who performed the kick off
  BallContact *firstContact = ballContactHistory.at(1).get();
  if (touchBallKickoff != NULL
      and currentState->prevState != NULL
      and (currentState->prevState->name == "KickOff_Left" or currentState->prevState->name == "KickOff_Right")
      and touchBallKickoff->side == firstContact->side
      and touchBallKickoff->uNum == firstContact->uNum) {
    if (currentState->prevState->name == "KickOff_Left") {
      SetCurrent(kickOffRightState.get());
    } else {
      SetCurrent(kickOffLeftState.get());
    }
  }
}

/////////////////////////////////////////////////
void GameState::CheckCanScore()
{
  BallContact *ballContact = getLastBallContact();
  if (ballContact == NULL) {
    return;
  }
  for (size_t j = 0; j < teams.size(); j++) {
    Team *team = teams.at(j);
    if ((not team->canScore)
        and (touchBallKickoff != NULL)
        and ((ballContact->side != team->side)
             or (ballContact->side == team->side
                 and touchBallKickoff->uNum != ballContact->uNum
                 and ballContact->contactPos.Distance(SoccerField::CenterOfField) > SoccerField::CenterCircleRadius))) {
      team->canScore = true;
    }
  }
}


/////////////////////////////////////////////////
void GameState::MoveAgent(Agent &agent, const math::Vector3<double> &pos)
{
  agent.pos = pos;
  agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgent(Agent &agent, const double x, const double y, const double yaw)
{
  agent.pos.Set(x, y, GameState::beamHeight);
  agent.rot.Euler(0, 0, yaw);
  agent.updatePose = true;
}

/////////////////////////////////////////////////
void GameState::MoveAgentNoisy(Agent &agent, const double x, const double y, const double yaw)
{
  double offsetX = ((double) rand() / (RAND_MAX)) * 0.2 - 0.1;
  double offsetY = ((double) rand() / (RAND_MAX)) * 0.2 - 0.1;
  double offsetYaw = ((double) rand() / (RAND_MAX)) * 0.2 - 0.1;
  agent.pos.Set(x + offsetX, y + offsetY, GameState::beamHeight);
  agent.rot.Euler(0, 0, yaw + offsetYaw);
  agent.updatePose = true;
}

void GameState::MoveAgent(Agent &agent, const math::Vector3<double> &pos, const math::Quaternion<double> &rot)
{
  agent.pos = pos;
  agent.rot = rot;
  agent.updatePose = true;
}

void GameState::MoveAgentToSide(Agent &agent)
{
  double newY;
  if (agent.pos.Y() > 0) {
    newY = -SoccerField::HalfFieldHeight;
  } else {
    newY = SoccerField::HalfFieldHeight;
  }
  agent.pos = math::Vector3<double>(agent.pos.X(), newY, GameState::beamHeight);
  agent.updatePose = true;
}

/////////////////////////////////////////////////
math::Vector3<double> GameState::GetBall()
{
  return ballPos;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCenter()
{
  ballPos = SoccerField::BallCenterPosition;
  updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallForGoalKick()
{
  double newX = SoccerField::HalfFieldWidth - 1.0;
  if (ballPos.X() < 0) {
    newX = -newX;
  }
  ballPos = math::Vector3<double>(newX, 0, SoccerField::BallRadius);
  updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBallToCorner()
{
  ballPos = math::Vector3<double>(
              (fabs(ballPos.X()) / ballPos.X()) * SoccerField::HalfFieldWidth,
              (fabs(ballPos.Y()) / ballPos.Y()) * SoccerField::HalfFieldHeight,
              SoccerField::BallRadius);
  updateBallPose = true;
}


/////////////////////////////////////////////////
void GameState::MoveBallInBounds()
{
  double newX = std::max(std::min(SoccerField::HalfFieldWidth, ballPos.X()), -SoccerField::HalfFieldWidth);
  double newY = std::max(std::min(SoccerField::HalfFieldHeight, ballPos.Y()), -SoccerField::HalfFieldHeight);
  ballPos.Set(newX, newY, SoccerField::BallRadius);
  updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::MoveBall(const math::Vector3<double> &_ballPos)
{
  ballPos = _ballPos;
  updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::setBallVel(const math::Vector3<double> &_ballVel)
{
  ballVel = _ballVel;
  updateBallPose = true;
}

/////////////////////////////////////////////////
void GameState::setBallAngVel(const math::Vector3<double> &_ballAngVel)
{
  ballAngVel = _ballAngVel;
  updateBallPose = true;
}

////////////////////////////////////////////////
void GameState::Initialize()
{
  //do something when play mode changes
  //for now, do nothing
}

////////////////////////////////////////////////
bool GameState::addAgent(int uNum, std::string teamName)
{
  // std::cout << "adding agent: " << uNum << " teamName: " << teamName << std::endl;
  if (uNum < 0 or uNum > GameState::playerLimit) {
    std::cout << "uNum " << uNum << " is invalid, cannot add agent to team " << teamName << "!" << std::endl;
    return false;
  }

  int teamToAdd = -1;
  for (size_t i = 0; i < teams.size(); ++i) {
    if (teams.at(i)->name == teamName) {
      teamToAdd = i;
    }
  }
  if (teamToAdd == -1) {
    for (size_t i = 0; i < teams.size(); ++i) {
      if (teams.at(i)->name == "_unnamed_team" and teams.at(i)->members.size() == 0) {
        teamToAdd = i;
        teams.at(i)->name = teamName;
        break;
      }
    }
  }
  if (teamToAdd == -1) {
    // std::cout << uNum << " " << teamName.c_str() << std::endl;
    std::cout << "There already are two teams, cannot add agent into new team!" << std::endl;
    return false;
  }

  Team *team = teams.at(teamToAdd);
  if (static_cast<int>(team->members.size()) > GameState::playerLimit) {
    std::cout << "Team is already full, cannot add agent to team!" << std::endl;
    return false;
  }
  bool *uNumArray = new bool[GameState::playerLimit];
  for (int i = 0; i < GameState::playerLimit; i++) {
    uNumArray[i] = true;
  }
  for (size_t i = 0; i < team->members.size(); i++) {
    uNumArray[team->members.at(i).uNum - 1] = false;
    if (uNum != 0 and team->members.at(i).uNum == uNum) {
      std::cout << "Already have an agent with this unum: " << uNum << ", cannot add agent to team!" << std::endl;
      return false;
    }
  }
  if (uNum == 0) {
    for (int i = 0; i < GameState::playerLimit; i++) {
      if (uNumArray[i]) {
        uNum = i + 1;
        break;
      }
    }
  }
  if (uNum == 0) {
    std::cout << "No free uNums avaliable, cannot add agent to team!" << std::endl;
    return false;
  }
  delete[] uNumArray;
  team->members.push_back(Agent(uNum, team));
  return true;
}


///////////////////////////////////////////////
bool GameState::removeAgent(int uNum, std::string teamName)
{
  for (size_t i = 0; i < teams.size(); ++i) {
    if (teams.at(i)->name == teamName) {
      for (size_t j = 0; j < teams.at(i)->members.size(); j++) {
        if (teams.at(i)->members.at(j).uNum == uNum) {
          teams.at(i)->members.erase(teams.at(i)->members.begin() + j);
          return true;
        }
      }
    }
  }
  std::cout << "Agent not found, unable to remove agent!" << std::endl;
  return false;
}

////////////////////////////////////////////////
bool GameState::beamAgent(int uNum, std::string teamName, double x, double y, double rot)
{
  if (currentState->name != "BeforeKickOff" or currentState->name != "goal_kick_left" or currentState->name != "goal_kick_right") {
    std::cout << "Incorrect play mode, unable to beam agent!" << std::endl;
    return false;
  }
  for (size_t i = 0; i < teams.size(); ++i) {
    if (teams.at(i)->name == teamName) {
      for (size_t j = 0; j < teams.at(i)->members.size(); j++) {
        if (teams.at(i)->members.at(j).uNum == uNum) {
          MoveAgent(teams.at(i)->members.at(j), x, y, rot);
          return true;
        }
      }
    }
  }
  std::cout << "Agent not found, unable to beam agent!" << std::endl;
  return false;
}
