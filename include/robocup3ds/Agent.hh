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
#include <utility>
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

/// \brief Typedef for map of agent's body parts and positions
typedef std::map<std::string, ignition::math::Vector3<double> >
  AgentBodyMap;

/// \brief Typedef for uNum, teamName pairs for identifying agents
typedef std::pair<int, std::string> AgentId;

/// \brief Container that contains info for hear perceptor
class AgentHear
{
  /// \brief AgentHear constructor
  public: AgentHear():
    gameTime(-1),
    yaw(-1),
    self(false),
    isValid(false)
  {}

  /// \brief Time when the message was sent
  public: double gameTime;

  /// \brief Relative angle of message
  public: double yaw;

  /// \brief Whether message is broadcast by self
  public: bool self;

  /// \brief Message string
  public: std::string msg;

  /// \brief Whether message is valid and we should send
  public: bool isValid;
};

/// \brief This class serves as an container for the information sent to
/// the agent
class AgentPerceptions
{
  /// \brief AgentPerception constructor
  public: AgentPerceptions()
  {
    this->fieldLines.reserve(21);
  }

  /// \Brief Map of landmarks that have been transformed to agent's cood
  /// frame
  public: std::map<std::string, ignition::math::Vector3<double> > landMarks;

  /// \Brief Vector of lines that have been transformed to agent's cood
  /// frame
  public: std::vector<ignition::math::Line3<double> > fieldLines;

  /// \Brief Map of agent's perceptions of other agent's body parts
  /// Implemented as a nested map
  public: std::map<AgentId, AgentBodyMap> otherAgentBodyMap;

  /// \Brief Hear perceptor
  public: AgentHear hear;

  /// \Brief Map of hinge joints and their angles
  public: std::map<std::string, double> hingeJoints;
};

/// \brief This class serves as an container for the information by the
/// the agent
class AgentActions
{
  /// \brief Constructor
  public: AgentActions() {}

  /// \brief Stores the velocity and angle information
  public: std::map<std::string, double> jointEffectors;
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

  /// \brief Map of agent body parts in world coordinates
  public: AgentBodyMap selfBodyMap;

  /// \brief Container for an agent's perceptions
  public: AgentPerceptions percept;

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

/// \brief Container that contains info for say effector
class AgentSay
{
  /// \brief AgentSay constructor
  public: AgentSay():
    agentId(std::make_pair(-1, "")),
    isValid(false)
  {}

  /// \brief AgentId of agent who said message
  public: AgentId agentId;

  /// \brief Where the agent said the message
  public: ignition::math::Vector3<double> pos;

  /// \brief Message string
  public: std::string msg;

  /// \brief Whether message is valid
  public: bool isValid;
};

/// \brief Struct for helping to sort agents by their distances,
/// used by CheckCrowding_helper only
class AgentDist
{
  /// \brief Pointer to agent object
  public: Agent *agent;

  /// \brief Distance agent from a target
  public: double dist;
};

#endif
