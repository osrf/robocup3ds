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

#ifndef _GAZEBO_GAMESTATE_HH_
#define _GAZEBO_GAMESTATE_HH_

#include <ignition/math.hh>
#include <map>
#include <memory>
#include <vector>
#include <string>

#include "robocup3ds/Geometry.hh"
#include "robocup3ds/Agent.hh"

namespace states
{
  class BeforeKickOffState;
  class CornerKickState;
  class FreeKickState;
  class GameOverState;
  class GoalKickState;
  class GoalState;
  class KickInState;
  class KickOffState;
  class PlayOnState;
  class State;
}

/// \brief Class for controlling play mode transitions and checking rule
/// violations in a 3D simulation Robocup game.
///
/// To use GameState object, you need perform the following actions
/// repeatedly until the game is over:
/// 1) Use the simulation world model to update the pose of all
/// the players and ball. Also, check for collisions in simulation world model
/// and update ballContactHistory member variable if necessary
/// 2) Call the Effector object's Update() method
/// 3) Call the Update() method
/// 4) Call the Perceptor object's Update() method
/// 5) If the GameState object modifies the pose of any of the players and or
/// ball (by checking the updatePose flag), update the simulation world model
/// to match poses stored in the GameState object
class GameState
{
  /// \brief Enum for which half it is
  public: enum class Half
  {
    /// \brief First half of game
    FIRST_HALF,
    /// \brief Second half of game
    SECOND_HALF
  };

  /// \brief Struct for helping to sort agents by their distances,
  /// used by CheckCrowding_helper only
  private: class AgentDist
  {
    /// \brief Pointer to agent object
    public: Agent *agent;

    /// \brief Distance agent from a target
    public: double dist;
  };

  /// \brief Internal Logger for GameState
  private: class Logger
  {
    /// \brief Constructor for the logger class
    /// \param[in] _gameState Pointer to GameState object
    /// \param[in] _normalLevel Normal message logging level
    /// \param[in] _errorLevel Error messages above this level will be printed
    public: Logger(const GameState *const _gameState,
      const int _normalLevel,
                    const int _errorLevel):
        gameState(_gameState),
        normalLevel(_normalLevel),
        errorLevel(_errorLevel)
    {}

    /// \brief Print normal messages plus some game information
    /// \param[in] _message Normal message string
    /// \param[in] _level Normal messages above this level will be printed
    public: void Log(const std::string &_message, const int _level) const
    {
      if (_level >= this->normalLevel)
      {
        std::cout << "[" << this->gameState->GetCycleCounter() << "]["
                  << this->gameState->GetGameTime() << "][lvl:" <<
                  _level << "]\t" << _message.c_str();
      }
    }

    /// \brief Print error messages plus some game information
    /// \param[in] _message Error message string
    /// \param[in] _level Error messages above this level will be printed
    public: void LogErr(const std::string &_message, const int _level) const
    {
      if (_level >= this->errorLevel)
      {
        std::cout << "\033[31m[" << this->gameState->GetCycleCounter() << "]["
                  << this->gameState->GetGameTime() << "][lvl:" <<
                  _level << "]\t" << _message.c_str();
      }
    }

    /// \brief Pointer to parent gamestate object
    private: const GameState *const gameState;

    /// \brief Level for logging normal messages
    public: const int normalLevel;

    /// \brief Level for logging error messages
    public: const int errorLevel;
  };

  /// \brief Stores ball contact information for GameState
  public: class BallContact
  {
    /// \brief Constructor for Ball Contact object
    /// \param[in] _uNum agent unique id
    /// \param[in] _side side which touched ball
    /// \param[in] _contactTime time when ball was touched
    /// \param[in] _contactPos position where ball was touched
    public: BallContact(const int _uNum, const Team::Side _side,
                          const double _contactTime,
                          const ignition::math::Vector3<double> &_contactPos):
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
  /// \param[in] _config Map of configuration variables
  public: void LoadConfiguration(
    const std::map<std::string, std::string> &_config) const;

  /// \brief Destructor.
  public: virtual ~GameState();

  /// \brief Clears the history of ball contacts
  public: void ClearBallContactHistory();

  /// \brief Update the robocup simulation state.
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
  /// \param[in] _newState New state to replace current state
  /// \param[in] _resetStage When new state is the same as current state,
  /// setting this flag to true will reset the state
  public: void SetCurrent(const std::shared_ptr<states::State> &_newState,
    const bool _resetStage = false);

  /// \brief Drops the ball at its current position and move all players away
  /// by the free kick radius. If the ball is off the field, it is brought
  /// back within bounds.
  /// \param[in] _teamAllowed Team::LEFT if the left team is allowed to be
  /// close to the ball. Team::RIGHT if the right team is allowed or any other
  /// number if none of the teams are allowed to be within the free kick
  /// radius.
  public: void DropBallImpl(const Team::Side _teamAllowed);

  /// \brief Checks whether the double touching rules are violated
  public: void CheckDoubleTouch();

  /// \brief Check whether scoring conditions are met (another agent on
  /// same side that is not the kick off agent has touched the ball).
  public: void CheckCanScore();

  /// \brief Check that agents stay on their own side during kick offs
  /// \param[in] _kickingSide The kicking side during the kickoff
  public: void CheckOffSidesOnKickOff(const Team::Side _kickingSide);

  /// \brief Set the agent's position only
  /// \param[out] _agent Reference to agent object
  /// \param[in] _pos New position of agent
  public: void MoveAgent(Agent &_agent,
                         const ignition::math::Vector3<double> &_pos) const;

  /// \brief Set the agent's position and yaw (beaming the agent)
  /// \param[out] _agent Reference to agent object
  /// \param[in] _x New X position of agent
  /// \param[in] _y New Y position of agent
  /// \param[in] yaw New yaw of agent
  public: void MoveAgent(Agent &_agent, const double _x, const double _y,
                         const double yaw) const;

  /// \brief Set the agent's position and yaw with some additional noise
  /// \param[out] _agent Reference to agent object
  /// \param[in] _x New X position of agent
  /// \param[in] _y New Y position of agent
  /// \param[in] _yaw New yaw of agent
  public: void MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                              const double _yaw) const;

  /// \brief Set the agent's position and orientation
  /// \param[out] _agent Reference to agent object
  /// \param[in] _pos New position of agent
  /// \param[in] _rot New orientation of agent
  public: void MoveAgent(Agent &_agent,
                         const ignition::math::Vector3<double> &_pos,
                         const ignition::math::Quaternion<double> &_rot) const;

  /// \brief Move agent to side of field, the side depends on the agent's side
  /// \param[out] _agent Reference to agent object
  public: void MoveAgentToSide(Agent &_agent) const;

  /// \brief Move agent back to their own side if they are offsides during kick
  /// offs
  /// \param[out] _agent Reference to agent object
  public: void MoveOffSideAgent(Agent &_agent) const;

  /// \brief Check if the first half or the game ends, also update
  /// elapsed time.
  public: void CheckTiming();

  /// \brief Check the ball's position looking for goals or out of bounds.
  public: void CheckBall();

  /// \brief Checks that during goal kick, no members of opposing team
  /// are inside penalty area
  /// \param[in] _teamAllowed Side of the team that is allowed in penalty area
  public: void CheckGoalKickIllegalDefense(const Team::Side _teamAllowed);

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
  /// \param[in] _ballPos Target position.
  public: void MoveBall(const ignition::math::Vector3<double> &_ballPos);

  /// \brief Set the linear velocity of the ball
  /// \param[in] _ballVel Target linear velocity
  public: void SetBallVel(const ignition::math::Vector3<double> &_ballVel);

  /// \brief Set the angular velocity of the ball
  /// \param[in] _ballAngVel Target angular velocity
  public: void SetBallAngVel(const ignition::math::Vector3<double>
                             &_ballAngVel);

  /// \brief Add agent to game state
  /// \param[in] _uNum Agent number
  /// \param[in] _teamName Agent name
  /// \return True when adding agent is successful
  public: bool AddAgent(const int _uNum, const std::string &_teamName);

  /// \brief Remove agent from game state
  /// \param[in] _uNum Agent number
  /// \param[in] _teamName Agent name
  /// \return True when removing agent is successful
  public: bool RemoveAgent(const int _uNum, const std::string &_teamName);

  /// \brief Beam the agent if the play mode allows it
  /// \param[in] _uNum Agent number
  /// \param[in] _teamName Agent name
  /// \param[in] _x X position of agent
  /// \param[in] _y Y position of agent
  /// \param[in] _rot Yaw of agent
  /// \return Flag whether removing agent is successful
  public: bool BeamAgent(const int _uNum, const std::string &_teamName,
                         const double _x, const double _y, const double _rot);

  // Getter and setter methods

  /// \brief Get the game's half.
  /// \return if the game is in the first half or if is in the second.
  public: Half GetHalf() const;

  /// \brief Set the game half.
  /// \param[in] _newHalf Set the current half
  public: void SetHalf(const Half _newHalf);

  /// \brief Get the elapsed time since beginning of half.
  /// \return The elapsed game time
  public: double GetElapsedGameTime() const;

  /// \brief Get the elapsed time since last cycle.
  /// \return The elapsed game time since last cycle
  public: double GetElapsedCycleGameTime() const;

  /// \brief Set the game time.
  /// \param[in] _gameTime New game time
  public: void SetGameTime(const double _gameTime);

  /// \brief Get the game time.
  public: double GetGameTime() const;

  /// \brief Get the game time when play starts
  public: double GetStartGameTime() const;

  /// \brief Set the game time when play starts
  /// \param[in] _startGameTime New start game time
  public: void SetStartGameTime(const double _startGameTime);

  /// \brief Set the cycle counter
  /// \param[in] _cycleCounter New cycle counter
  public: void SetCycleCounter(const int _cycleCounter);

  /// \brief Get the cycle counter
  public: int GetCycleCounter() const;

  /// \brief Get the current state
  public: std::shared_ptr<states::State> GetCurrentState() const;

  /// \brief Get the side of the team who last touched the ball.
  /// \return Pointer to the side of the team
  public: Team::Side GetLastSideTouchedBall() const;

  /// \brief Get the last ball contact
  /// \return Pointer to the last ball contact
  public: std::shared_ptr<GameState::BallContact> GetLastBallContact() const;

  /// \brief Method executed each time the game state changes.
  private: void Initialize();

  /// \brief Comparison method for sorting AgentDist
  /// \param[in] _i First comparison AgentDist object
  /// \param[in] _j Second comparison AgentDist object
  /// \return True when _i is smaller than _j
  private: static bool SortDist(const AgentDist &_i, const AgentDist &_j);

  /// \brief Helper function for loading gameState configuration variables
  /// \param[in] _config Map of configuration variables
  /// \param[in] _key Key to look for in map
  /// \param[out] _value Value to return
  /// \return True if loading of parameter is successful
  private: bool LoadConfigParameter(
    const std::map<std::string, std::string> &_config,
    const std::string &_key, double &_value) const;

  /// \brief Helper function for loading gameState configuration variables
  /// \param[in] _config Map of configuration variables
  /// \param[in] _key Key to look for in map
  /// \param[out] _boolValue Value to return
  /// \return True if loading of parameter is successful
  private: bool LoadConfigParameterBool(
    const std::map<std::string, std::string> &_config,
    const std::string &_key, bool &_boolValue) const;

  // member and class variables

  /// \brief beforeKickOffState playmode
  public: std::shared_ptr<states::BeforeKickOffState> beforeKickOffState;

  /// \brief kickOffLeftState playmode
  public: std::shared_ptr<states::KickOffState> kickOffLeftState;

  /// \brief kickOffRightState playmode
  public: std::shared_ptr<states::KickOffState> kickOffRightState;

  /// \brief playState playmode
  public: std::shared_ptr<states::PlayOnState> playOnState;

  /// \brief kickInLeftState playmode
  public: std::shared_ptr<states::KickInState> kickInLeftState;

  /// \brief kickInRightState playmode
  public: std::shared_ptr<states::KickInState> kickInRightState;

  /// \brief cornerKickLeftState playmode
  public: std::shared_ptr<states::CornerKickState> cornerKickLeftState;

  /// \brief cornerKickRightState playmode
  public: std::shared_ptr<states::CornerKickState> cornerKickRightState;

  /// \brief goalKickLeftState playmode
  public: std::shared_ptr<states::GoalKickState> goalKickLeftState;

  /// \brief goalKickRightState playmode
  public: std::shared_ptr<states::GoalKickState> goalKickRightState;

  /// \brief gameOverState playmode
  public: std::shared_ptr<states::GameOverState> gameOverState;

  /// \brief goalLeftState playmode
  public: std::shared_ptr<states::GoalState> goalLeftState;

  /// \brief goalRightState playmode
  public: std::shared_ptr<states::GoalState> goalRightState;

  /// \brief freeKickLeftState playmode
  public: std::shared_ptr<states::FreeKickState> freeKickLeftState;

  /// \brief freeKickRightState playmode
  public: std::shared_ptr<states::FreeKickState> freeKickRightState;

  /// \brief Name of BeforeKickOff playmode
  public: static const std::string BeforeKickOff;

  /// \brief Name of KickOffLeft playmode
  public: static const std::string KickOffLeft;

  /// \brief Name of KickOffRight playmode
  public: static const std::string KickOffRight;

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
  public: static std::shared_ptr<std::map<const std::string,
    const std::string>> config;

  /// \brief Whether currentState has changed in the current update cycle or not
  public: bool hasCurrentStateChanged;

  /// \brief History of ball contacts;
  public: std::vector<std::shared_ptr<BallContact>> ballContactHistory;

  /// \brief Pointer to the ball contact that causes the game to
  /// transition from kick off to play on
  public: std::shared_ptr<BallContact> touchBallKickoff;

  /// \brief All the teams.make
  public: std::vector <std::shared_ptr<Team>> teams;

  /// \brief Flag whether to update ball position in world to match game state.
  public: bool updateBallPose;

  /// \brief Position of soccer ball.
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
  private: std::shared_ptr<states::State> currentState;

  /// \brief Game half (1st half or 2nd half).
  private: Half half;

  /// \brief Number of cycles of updates elapsed
  private: int cycleCounter;
};

#endif
