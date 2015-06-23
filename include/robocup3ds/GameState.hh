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

#ifndef _GAZEBO_ROBOCUP_3DS_PLUGIN_HH_
#define _GAZEBO_ROBOCUP_3DS_PLUGIN_HH_

#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <string>
#include <memory>
#include <map>
#include <ignition/math.hh>

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

/// \class GameState GameState.hh robocup3ds/GameState.hh
/// \brief Class for controlling play mode transitions and checking rule
/// violations in a 3d simulation Robocup game
class GameState
{
  public: class Agent;
  public: class Team;
  public: class BallContact;
  /// \brief Enum for which half it is
  public: enum Half {FIRST_HALF, SECOND_HALF};

  /// \Brief Struct for helping to sort agents by their distances,
  /// used by CheckCrowding_helper only
  private: struct AgentDist
  {
    Agent *agent;
    double dist;
  };

  /// \class DataLogger GameState.hh robocup3ds/GameState.hh
  /// \brief Internal Logger for GameState
  private: class Logger {
    /// \brief Constructor for the logger class
    /// \param[in] Pointer to GameState object
    /// \param[in] Normal message logging level
    /// \param[in] Error messages above this level will be printed
    public: Logger(GameState *_gameState, int _normalLevel,
                     int _errorLevel):
        gameState(_gameState),
        normalLevel(_normalLevel),
        errorLevel(_errorLevel)
    {}

    /// \brief Print normal messages plus some game information
    /// \param[in] Normal message string
    /// \param[in] Normal messages above this level will be printed
    public: void Log(std::string _message, int _level)
    {
      if (_level >= normalLevel)
      {
        std::cout << "[" << gameState->cycleCounter << "]["
                  << gameState->GetGameTime() << "][lvl:" <<
                  _level << "]\t" << _message.c_str();
      }
    }

    /// \brief Print error messages plus some game information
    /// \param[in] Error message string
    /// \param[in] Error messages above this level will be printed
    public: void LogErr(std::string _message, int _level)
    {
      if (_level >= errorLevel)
      {
        std::cout << "\033[31m[" << gameState->cycleCounter << "]["
                  << gameState->GetGameTime() << "][lvl:" <<
                  _level << "]\t" << _message.c_str();
      }
    }

    private: GameState *gameState;

    public: int normalLevel, errorLevel;
  };

  /// \class Team GameState.hh robocup3ds/GameState.hh
  /// \brief Team class for GameState
  public: class Team
  {
    /// \brief Enum for the team side
    public: enum Side {NEITHER = -1, LEFT, RIGHT };

    /// \brief Constructor for Team object
    /// \param[in] Name of team
    /// \param[in] Side of team
    /// \param[in] Starting score of team
    /// \param[in] Maximum players on team
    public: Team(std::string _name, Side _side, int _score, int _playerLimit):
      name(_name),
      side(_side),
      score(_score)
    {
      this->members.reserve(_playerLimit);
      this->numPlayersInPenaltyBox = 0;
      this->canScore = false;
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

  /// \class Agent GameState.hh robocup3ds/GameState.hh
  /// \brief Agent class for GameState
  public: class Agent
  {
    /// \brief Enum for the agent status
    public: enum AgentStatus {RELEASED, STOPPED };

    /// \brief Constructor for Agent object
    /// \param[in] unique identifier for agent
    /// \param[in] pointer to team agent is on
    public: Agent(int _uNum, std::shared_ptr<Team> _team):
      uNum(_uNum),
      team(_team)
    {
      this->pos.Set(0, 0, GameState::beamHeight);
      this->prevPos.Set(0, 0, GameState::beamHeight);
      this->status = RELEASED;
      this->updatePose = false;
      this->inPenaltyBox = false;
      this->timeImmoblized = 0;
      this->timeFallen = 0;
    }

    /// \brief Agent unique id
    public: int uNum;
    /// \brief Pointer to team that agent belongs to
    public: std::shared_ptr<Team> team;
    /// \brief Agent status
    public: AgentStatus status;
    /// \brief Agent position
    public: ignition::math::Vector3<double> pos;
    /// \brief Agent position
    public: ignition::math::Vector3<double> prevPos;
    /// \brief Agent rotation
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

  /// \class BallContact GameState.hh robocup3ds/GameState.hh
  /// \brief Stores ball contact information for GameState
  public: class BallContact
  {
    /// \brief Constructor for Ball Contact object
    /// \param[in] agent unique id
    /// \param[in] side which touched ball
    /// \param[in] time when ball was touched
    /// \param[in] position where ball was touched
    public: BallContact(int _uNum, Team::Side _side, double _contactTime,
                          ignition::math::Vector3<double> _contactPos):
      uNum(_uNum),
      side(_side),
      contactTime(_contactTime),
      contactPos(_contactPos)
    {}

    /// \brief Unum of agent who touched ball
    public: int uNum;
    /// \brief Side of agent who touched ball
    public: Team::Side side;
    /// \brief Time when agent stopped contacting the ball
    /// (if contact is longer than an instant)
    public: double contactTime;
    /// \brief Position where agent contacted ball
    public: ignition::math::Vector3<double> contactPos;
    /// \brief Pointer to team of agent who touched ball
    public: std::string teamName;
  };

  // methods and constructors

  /// \brief Constructor.
  public: GameState();

  /// \brief Function for loading gameState configuration variables
  void LoadConfiguration();

  /// \brief Destructor.
  public: virtual ~GameState();

  /// \Clears the history of ball contacts
  public: void ClearBallContactHistory();

  /// \brief Update the robocup simulation state.
  /// \param[in] _info Information used in the update event.
  public: void Update();

  /// \brief During some of the states of the game the players are not allowed
  /// to move (for example during kickoff). This method releases the players
  /// when it's time to move the robots (for example in play mode).
  public: void ReleasePlayers();

  /// \brief During some of the states of the game the players are not allowed
  /// to move (for example during kickoff). This method stops the players by
  /// creating a joint to the world for each player.
  public: void StopPlayers();

  /// \brief Set the current game state. If the new state is the same than
  /// the current one, the operation does not have any effect.
  /// \param[in] new state to replace current state
  public: void SetCurrent(State *_newState);

  /// \brief Drops the ball at its current position and move all players away
  /// by the free kick radius. If the ball is off the field, it is brought
  /// back within bounds.
  /// \param[in] Team::LEFT if the left team is allowed to be close to the
  /// ball. Team::RIGHT if the right team is allowed or any other number if
  /// none of the teams are allowed to be within the free kick radius.
  public: void DropBallImpl(const Team::Side _teamAllowed);

  /// \brief Checks whether the double touching rules are violated
  public: void CheckDoubleTouch();

  /// \brief Check whether scoring conditions are met (another agent on
  /// same side that is not the kick off agent has touched the ball).
  public: void CheckCanScore();

  /// \brief Check that agents stay on their own side during kick offs
  /// \param[in] The kicking side during the kickoff
  public: void CheckOffSidesOnKickOff(Team::Side _kickingSide);

  /// \brief Set the agent's position only
  /// \param[in] Reference to agent object
  /// \param[in] New position of agent
  public: void MoveAgent(Agent &_agent,
                           const ignition::math::Vector3<double> &_pos);

  /// \brief Set the agent's position and yaw (beaming the agent)
  /// \param[in] Reference to agent object
  /// \param[in] New X position of agent
  /// \param[in] New Y position of agent
  /// \param[in] New yaw of agent
  public: void MoveAgent(Agent &_agent, const double _x, const double _y,
                           const double yaw);

  /// \brief Set the agent's position and yaw with some additional noise
  /// \param[in] Reference to agent object
  /// \param[in] New X position of agent
  /// \param[in] New Y position of agent
  /// \param[in] New yaw of agent
  public: void MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                                const double _yaw);

  /// \brief Set the agent's position and orientation
  /// \param[in] Reference to agent object
  /// \param[in] New position of agent
  /// \param[in] New orientation of agent
  public: void MoveAgent(Agent &_agent,
                           const ignition::math::Vector3<double> &_pos,
                           const ignition::math::Quaternion<double> &_rot);

  /// \brief Move agent to side of field, the side depends on the agent's side
  /// \param[in] Reference to agent object
  public: void MoveAgentToSide(Agent &_agent);

  /// \brief Move agent back to their own side if they are offsides during kick
  /// offs
  /// \param[in] Reference to agent object
  public: void MoveOffSideAgent(Agent &_agent);

  /// \brief Check if the first half or the game ends, also update
  /// elapsed time.
  public: void CheckTiming();

  /// \brief Check the ball's position looking for goals or out of bounds.
  public: void CheckBall();

  /// \brief Checks that during goal kick, no members of opposing team
  /// are inside penalty area
  /// \param[in] Side of the team that is allowed in penalty area
  public: void CheckGoalKickIllegalDefense(Team::Side _teamAllowed);

  /// \brief Check that no more than 3 players are in penalty area.
  public: void CheckIllegalDefense();

  /// \brief Check that no player has fallen or remain still for too long.
  public: void CheckImmobility();

  /// \brief Check crowding rules.
  public: void CheckCrowding();

  /// \brief Get the ball position in the field.
  /// \return The position of the ball.
  public: ignition::math::Vector3<double> GetBall();

  /// \brief Move the ball to the center of field.
  public: void MoveBallToCenter();

  /// \brief Move ball to goal kick position.
  public: void MoveBallForGoalKick();

  /// \brief Move the ball to the corner of the field.
  public: void MoveBallToCorner();

  /// \brief Move the ball in bounds if it is out of bounds.
  public: void MoveBallInBounds();

  /// \brief Move the ball to a given position.
  /// \param[in] Target position.
  public: void MoveBall(const ignition::math::Vector3<double> &_ballPos);

  /// \brief Set the linear velocity of the ball
  /// \param[in] Target linear velocity
  public: void SetBallVel(const ignition::math::Vector3<double> &_ballVel);

  /// \brief Set the angular velocity of the ball
  /// \param[in] Target angular velocity
  public: void SetBallAngVel(const ignition::math::Vector3<double>
                               &_ballAngVel);

  /// \brief Add agent to game state
  /// \param[in] Agent number
  /// \param[in] Agent name
  /// \return Flag whether adding agent is successful
  public: bool AddAgent(int _uNum, std::string _teamName);

  /// \brief Remove agent from game state
  /// \param[in] Agent number
  /// \param[in] Agent name
  /// \return Flag whether removing agent is successful
  public: bool RemoveAgent(int _uNum, std::string _teamName);

  /// \brief Beam the agent if the play mode allows it
  /// \param[in] unique identifier of agent
  /// \param[in] name of agent
  /// \param[in] X position of agent
  /// \param[in] Y position of agent
  /// \param[in] Yaw of agent
  /// \return Flag whether removing agent is successful
  public: bool BeamAgent(int _uNum, std::string teamName, double _x, double _y,
                           double _rot);

  // Getter and setter methods

  /// \brief Get the game's half.
  /// \return if the game is in the first half or if is in the second.
  public: Half GetHalf()
  {
    return this->half;
  }

  /// \brief Set the game half.
  /// \param[in] _newHalf 1 for first half or 2 for second half.
  public: void SetHalf(Half _newHalf)
  {
    this->half = _newHalf;
  }

  /// \brief Get the elapsed time since beginning of half.
  /// \return The elapsed game time
  public: double GetElapsedGameTime()
  {
    return this->gameTime - this->startGameTime;
  }

  /// \brief Get the elapsed time since last cycle.
  /// \return The elapsed game time since last cycle
  public: double GetElapsedCycleGameTime()
  {
    return this->gameTime - this->prevCycleGameTime;
  }

  /// \brief Set the game time.
  /// \param[in] New game time
  public: void SetGameTime(double _gameTime)
  {
    this->gameTime = _gameTime;
  }

  /// \brief Get the game time.
  public: double GetGameTime()
  {
    return this->gameTime;
  }

  /// \brief Get the game time when play starts
  public: double GetStartGameTime()
  {
    return this->startGameTime;
  }

  /// \brief Set the game time when play starts
  /// \param[in] New start game time
  public: void SetStartGameTime(double _startGameTime)
  {
    this->startGameTime = _startGameTime;
  }

  /// \brief Set the cycle counter
  /// \param[in] New cycle counter
  public: void SetCycleCounter(int _cycleCounter)
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

  /// \brief Get the cycle counter
  public: int GetCycleCounter()
  {
    return this->cycleCounter;
  }

  /// \brief Get the current state
  public: State *GetCurrentState()
  {
    return this->currentState;
  }

  /// \brief Get the side of the team who last touched the ball.
  /// \return Pointer to the side of the team
  public: Team::Side *GetLastSideTouchedBall()
  {
    if (this->GetLastBallContact() != NULL)
    {
      return &(this->GetLastBallContact()->side);
    }
    else
    {
      return NULL;
    }
  }

  /// \brief Get the last ball contact
  /// \return Pointer to the last ball contact
  public: BallContact *GetLastBallContact() {
    if (this->ballContactHistory.size() > 0)
    {
      return this->ballContactHistory.at(
               this->ballContactHistory.size() - 1).get();
    }
    else
    {
      return NULL;
    }
  }

  /// \brief Method executed each time the game state changes.
  private: void Initialize();

  /// \brief Comparison method for sorting AgentDist
  private: static bool SortDist(AgentDist _i, AgentDist _j)
  {
    return (_i.dist < _j.dist);
  }
  /// \brief Helper function for loading gameState configuration variables
  private: bool LoadConfigParameter(const std::string &_key, double &_value);

  /// \brief Helper function for loading gameState configuration variables
  private: bool LoadConfigParameterBool(const std::string &_key,
    bool &_boolValue);

  // member and class variables

  /// \brief beforeKickOffState playmode
  public: std::shared_ptr<BeforeKickOffState> beforeKickOffState;
  /// \brief kickOffLeftState playmode
  public: std::shared_ptr<KickOffLeftState> kickOffLeftState;
  /// \brief kickOffRightState playmode
  public: std::shared_ptr<KickOffRightState> kickOffRightState;
  /// \brief playState playmode
  public: std::shared_ptr<PlayOnState> playOnState;
  /// \brief kickInLeftState playmode
  public: std::shared_ptr<KickInLeftState> kickInLeftState;
  /// \brief kickInRightState playmode
  public: std::shared_ptr<KickInRightState> kickInRightState;
  /// \brief cornerKickLeftState playmode
  public: std::shared_ptr<CornerKickLeftState> cornerKickLeftState;
  /// \brief cornerKickRightState playmode
  public: std::shared_ptr<CornerKickRightState> cornerKickRightState;
  /// \brief goalKickLeftState playmode
  public: std::shared_ptr<GoalKickLeftState> goalKickLeftState;
  /// \brief goalKickRightState playmode
  public: std::shared_ptr<GoalKickRightState> goalKickRightState;
  /// \brief gameOverState playmode
  public: std::shared_ptr<GameOverState> gameOverState;
  /// \brief goalLeftState playmode
  public: std::shared_ptr<GoalLeftState> goalLeftState;
  /// \brief goalRightState playmode
  public: std::shared_ptr<GoalRightState> goalRightState;
  /// \brief freeKickLeftState playmode
  public: std::shared_ptr<FreeKickLeftState> freeKickLeftState;
  /// \brief freeKickRightState playmode
  public: std::shared_ptr<FreeKickRightState> freeKickRightState;
  /// \brief Name of BeforeKickOff playmode
  public:  static const std::string BeforeKickOff;
  /// \brief Name of KickOffLeft playmode
  public:  static const std::string KickOffLeft;
  /// \brief Name of KickOffRight playmode
  public:  static const std::string KickOffRight;
  /// \brief Name of PlayOn playmode
  public: static const std::string PlayOn;
  /// \brief Name of KickInLeft playmode
  public: static const std::string KickInLeft;
  /// \brief Name of KickInRight playmode
  public: static const std::string KickInRight;
  /// \brief Name of CornerKickLeft playmode
  public: static const std::string CornerKickLeft;
  /// \brief Name of CornerKickRight playmode
  public: static const std::string CornerKickRight;
  /// \brief Name of GoalKickLeft playmode
  public: static const std::string GoalKickLeft;
  /// \brief Name of GoalKickRight playmode
  public: static const std::string GoalKickRight;
  /// \brief Name of GameOver playmode
  public: static const std::string GameOver;
  /// \brief Name of GoalLeft playmode
  public: static const std::string GoalLeft;
  /// \brief Name of GoalRight playmode
  public: static const std::string GoalRight;
  /// \brief Name of FreeKickLeft playmode
  public: static const std::string FreeKickLeft;
  /// \brief Name of FreeKickRight playmode
  public: static const std::string FreeKickRight;

  /// \brief Duration of a complete game
  public: static double SecondsFullGame;
  /// \brief Duration of a half
  public: static double SecondsEachHalf;
  /// \brief Duration of pause after goal
  public: static double SecondsGoalPause;
  /// \brief Duration of pause after transition to kickIn
  public: static double SecondsKickInPause;
  /// \brief Duration of kickIn mode
  public: static double SecondsKickIn;
  /// \brief Duration of beforeKickOff mode
  public: static double SecondsBeforeKickOff;
  /// \brief Duration of kickOff mode
  public: static double SecondsKickOff;
  /// \brief Radius where not allowed team cannot approach ball
  public: static double dropBallRadius;
  /// \brief Every cycle counts as 20ms of game time (for `fake simulations)
  public: static bool useCounterForGameTime;
  /// \brief Max players per team
  public: static int playerLimit;
  /// \brief Max players in penalty box
  public: static int penaltyBoxLimit;
  /// \brief Beam height
  public: static double beamHeight;
  /// \brief Distance when to enable crowding rules
  public: static double crowdingEnableRadius;
  /// \brief Distance when to reposition one of two players
  public: static double innerCrowdingRadius;
  /// \brief Distance when to reposition one of three players
  public: static double outerCrowdingRadius;
  /// \brief Timeout when player remains immobile too long
  public: static double immobilityTimeLimit;
  /// \brief Timeout when player remains fallen too long
  public: static double fallenTimeLimit;

  /// \brief Pointer to configuration variables
  public: static std::map<const std::string, std::string> *config;
  /// \brief Whether currentState has changed in the current update cycle or not
  public: bool hasCurrentStateChanged;
  /// \brief History of ball contacts;
  public: std::vector<std::shared_ptr<BallContact> > ballContactHistory;
  /// \brief Pointer to the ball contact that causes the game to
  /// transition from kick off to play on
  public: BallContact *touchBallKickoff;
  /// \brief All the teams.
  public: std::vector <std::shared_ptr<Team> > teams;
  /// \brief Position of soccer ball.
  public: bool updateBallPose;
  /// \brief Flag whether to update ball position in world to match game state.

  private: ignition::math::Vector3<double> ballPos;
  /// \brief Angular velocity of soccer ball.
  private: ignition::math::Vector3<double> ballAngVel;
  /// \brief Linear velocity of soccer ball.
  private: ignition::math::Vector3<double> ballVel;
  /// \brief Game time.
  private: double gameTime;
  /// \brief Game time during previous cycle.
  private: double prevCycleGameTime;
  /// \brief Time when half starts (elapsed game time is essentially
  /// gameTime - startGameTime)
  private: double startGameTime;
  /// \brief Pointer to the current game state.
  private: State *currentState;
  /// \brief Game half (1st half or 2nd half).
  private: Half half;
  /// \brief Number of cycles of updates elapsed
  private: int cycleCounter;
};

#endif
