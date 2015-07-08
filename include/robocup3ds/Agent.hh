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

#ifndef _GAZEBO_AGENT_HH_
#define _GAZEBO_AGENT_HH_

#include <ignition/math.hh>
#include <map>
#include <memory>
#include <vector>
#include <string>

class Agent;

/// \brief Team class for GameState
class Team
{
  /// \brief Enum for the team side
  public: enum class Side
  {
    /// \brief Neither team/side of field
    NEITHER = -1,
    /// \brief Team on left side of field
    LEFT,
    /// \brief Team on right side of field
    RIGHT
  };

  /// \brief Constructor for Team object
  /// \param[in] _name Name of team
  /// \param[in] _side Side of team
  /// \param[in] _score Starting score of team
  /// \param[in] _playerLimit Maximum players on team
  public: Team(const std::string &_name, const Side _side,
    const int _score, const int _playerLimit):
    name(_name),
    side(_side),
    score(_score),
    numPlayersInPenaltyBox(0),
    canScore(false)
  {
    this->members.reserve(_playerLimit);
  }

  /// \brief Equality operator for teams
  /// \param[in] _team Tean compared against
  /// \return True if they are equal
  public: bool operator==(const Team &_team)
  {
    return this == &_team;
  }

  /// \brief Name of the team.
  public: std::string name;

  /// \brief All the members in the team.
  public: std::vector<Agent> members;

  /// \brief Side of the team
  public: Side side;

  /// \brief Team score
  public: int score;

  /// \brief Number players in penalty area
  public: int numPlayersInPenaltyBox;

  /// \brief Can score goal or not
  public: bool canScore;
};

/// \brief Agent class for GameState
class Agent
{
  /// \brief Enum for the agent status
  public: enum class Status
  {
    /// \brief Agent is not allowed to move
    RELEASED,
    /// \brief Agent is allowed to move
    STOPPED
  };

  /// \brief Constructor for Agent object
  /// \param[in] _uNum unique identifier for agent
  /// \param[in] _team pointer to team agent is on
  public: Agent(const int _uNum, const std::shared_ptr<Team> &_team):
    uNum(_uNum),
    team(_team)
  {
    this->pos.Set(0, 0, 0);
    this->prevPos.Set(0, 0, 0);
    this->status = Status::RELEASED;
    this->updatePose = false;
    this->inPenaltyBox = false;
    this->timeImmoblized = 0;
    this->timeFallen = 0;
  }

  /// \brief Equality operator for agents
  /// \param[in] _agent Agent compared against
  /// \return True if they are equal
  public: bool operator==(const Agent &_agent)
  {
    return this == &_agent;
  }

  /// \brief Agent unique id
  public: int uNum;

  /// \brief Pointer to team that agent belongs to
  public: std::shared_ptr<Team> team;

  /// \brief Agent status
  public: Status status;

  /// \brief Agent position
  public: ignition::math::Vector3<double> pos;

  /// \brief Agent position in previous cycle
  public: ignition::math::Vector3<double> prevPos;

  /// \brief Agent camera orientation
  public: ignition::math::Quaternion<double> cameraRot;

  /// \brief Agent orientation
  public: ignition::math::Quaternion<double> rot;

  /// \brief Flag whether to update agent pose in world to match
  /// gamestate.
  public: bool updatePose;

  /// \brief Flag whether agent is in penalty box
  public: bool inPenaltyBox;

  /// \brief Stores time the agent has not moved
  public: double timeImmoblized;

  /// \brief Stores time the agent has fallen
  public: double timeFallen;

  /// \brief Flag whether player is goalkeeper
  public: bool IsGoalKeeper() {
    return this->uNum == 1;
  }
};

#endif
