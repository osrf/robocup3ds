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

#include <ignition/math.hh>
#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>

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
#include "robocup3ds/states/PlayState.hh"

class GameState
{

    /// \brief Internal Logger for GameState
  private: class Logger
    {
      private: GameState *gameState;
      public: int normalLevel, errorLevel;

      public: Logger(GameState *_gameState, int _normalLevel, int _errorLevel):
          gameState(_gameState),
          normalLevel(_normalLevel),
          errorLevel(_errorLevel)
        {}

      public: void log(std::string message, int level)
        {
          if (level >= normalLevel) {
            std::cout << "[" << gameState->cycleCounter << "][lvl:" << level << "]\t" << message.c_str();
          }
        }

      public: void logErr(std::string message, int level)
        {
          if (level >= errorLevel) {
            std::cout << "\033[31m[" << gameState->cycleCounter << "][lvl:" << level << "]\t" << message.c_str();
          }
        }
    };

  public: class Agent;

    /// \brief Team class.
  public: class Team
    {
        /// \brief Enum for the team side
      public: enum Side {NEITHER = -1, LEFT, RIGHT};
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

        /// \brief Constructor for Team object
      public: Team(std::string _name, Side _side, int _score, int playerLimit):
          name(_name),
          side(_side),
          score(_score)
        {
          members.reserve(playerLimit);
          numPlayersInPenaltyBox = 0;
          canScore = false;
        }
    };

    /// \brief Agent class.
  public: class Agent
    {
        /// \brief Enum for the agent status
      public: enum AgentStatus {RELEASED, STOPPED};
        /// \brief Agent unique id
      public: int uNum;
        //   /// \brief Agent name
        // public: std::string name;
        /// \brief Pointer to team that agent belongs to
      public: Team *team;
        /// \brief Agent status
      public: AgentStatus status;
        /// \brief Agent position
      public: ignition::math::Vector3<double> pos;
        /// \brief Agent position
      public: ignition::math::Vector3<double> prevPos;
        /// \brief Agent rotation
      public: ignition::math::Quaternion<double> rot;
        /// \brief Flag whether to update agent pose in world to match gamestate.
      public: bool updatePose;
        /// \brief Flag whether agent is in penalty box
      public: bool inPenaltyBox;
        /// \brief Stores time the agent has not moved
      public: double timeImmoblized;
        /// \brief Stores time the agent has fallen
      public: double timeFallen;
        /// \brief Flag whether player is goalkeeper
      public: bool isGoalKeeper()
        {
          return uNum == 1;
        }

        /// \brief Constructor for Agent object
      public: 
        Agent(int _uNum, Team *_team):
          uNum(_uNum),
          team(_team)
        {
          pos.Set(0, 0, GameState::beamHeight);
          prevPos.Set(0, 0, GameState::beamHeight);
          status = RELEASED;
          updatePose = false;
          inPenaltyBox = false;
          timeImmoblized = 0;
          timeFallen = 0;
        }
    };

    /// \brief Class for storing ball contact information
  public: class BallContact
    {
      public:
        /// \brief Unum of agent who touched ball
        int uNum;
        /// \brief Side of agent who touched ball
        Team::Side side;
        /// \brief Time when agent stopped contacting the ball (if contact is longer than an instant)
        double contactTime;
        /// \brief Position where agent contacted ball
        ignition::math::Vector3<double> contactPos;
        /// \brief Pointer to team of agent who touched ball
        std::string teamName;

        /// \brief Constructor for Ball Contact object
        BallContact(int _uNum, Team::Side _side, double _contactTime, ignition::math::Vector3<double> _contactPos):
          uNum(_uNum),
          side(_side),
          contactTime(_contactTime),
          contactPos(_contactPos)
        {}
    };

    /*
    member and class variables
    */
  public:
    boost::shared_ptr<BeforeKickOffState> beforeKickOffState;
    boost::shared_ptr<KickOffLeftState> kickOffLeftState;
    boost::shared_ptr<KickOffRightState> kickOffRightState;
    boost::shared_ptr<PlayState> playState;
    boost::shared_ptr<KickInLeftState> kickInLeftState;
    boost::shared_ptr<KickInRightState> kickInRightState;
    boost::shared_ptr<CornerKickLeftState> cornerKickLeftState;
    boost::shared_ptr<CornerKickRightState> cornerKickRightState;
    boost::shared_ptr<GoalKickLeftState> goalKickLeftState;
    boost::shared_ptr<GoalKickRightState> goalKickRightState;
    boost::shared_ptr<GameOverState> gameOverState;
    boost::shared_ptr<GoalLeftState> goalLeftState;
    boost::shared_ptr<GoalRightState> goalRightState;
    boost::shared_ptr<FreeKickLeftState> freeKickLeftState;
    boost::shared_ptr<FreeKickRightState> freeKickRightState;

    //various play mode state names
    static const std::string BeforeKickOff;
    static const std::string KickOffLeft;
    static const std::string KickOffRight;
    static const std::string Play;
    static const std::string KickInLeft;
    static const std::string KickInRight;
    static const std::string CornerKickLeft;
    static const std::string CornerKickRight;
    static const std::string GoalKickLeft;
    static const std::string GoalKickRight;
    static const std::string GameOver;
    static const std::string GoalLeft;
    static const std::string GoalRight;
    static const std::string FreeKickLeft;
    static const std::string FreeKickRight;

    // values for various times
    static const double SecondsFullGame;
    static const double SecondsEachHalf;
    static const double SecondsGoalPause;
    static const double SecondsKickInPause;
    static const double SecondsKickIn;
    static const double SecondsBeforeKickOff;
    static const double SecondsKickOff;
    /// \brief Radius where not allowed team cannot approach ball
    static const double dropBallRadius;
    /// \brief Every cycle counts as 20ms of game time (for `fake simulations)
    static const bool useCounterForGameTime;
    /// \brief Max players per team
    static const int playerLimit;
    /// \brief Max players in penalty box
    static const int penaltyBoxLimit;
    /// \brief Beam height
    static const double beamHeight;
    /// \brief Distance when to enable crowding rules
    static const double crowdingEnableDist;
    /// \brief Distance when to reposition one of two players
    static const double crowdingReposDist2;
    /// \brief Distance when to reposition one of three players
    static const double crowdingReposDist3;
    /// \brief Timeout when player remains immobile too long
    static const double immobilityTimeLimit;
    /// \brief Timeout when player remains fallen too long
    static const double fallenTimeLimit;

/// \brief Enum for which half it is
  public: enum Half {FIRST_HALF, SECOND_HALF};
/// \brief Whether currentState has changed in the current update cycle or not
  public: bool hasCurrentStateChanged;
/// \brief History of ball contacts;
  public: std::vector<boost::shared_ptr<BallContact> > ballContactHistory;
/// \brief Pointer to the ball contact that causes the game to transition from kick off to play on
  public: BallContact *touchBallKickoff;
/// \brief All the teams.
  public: std::vector<Team *> teams;
/// \brief Position of soccer ball.
  private: ignition::math::Vector3<double> ballPos;
/// \brief Angular velocity of soccer ball.
  public: bool updateBallPose;
/// \brief Flag whether to update ball position in world to match game state.
  private: ignition::math::Vector3<double> ballAngVel;
/// \brief Linear velocity of soccer ball.
  private: ignition::math::Vector3<double> ballVel;
/// \brief Game time.
  private: double gameTime;
/// \brief Game time during previous cycle.
  private: double prevCycleGameTime;
/// \brief Time when half starts (elapsed game time is essentially gameTime - startGameTime)
  private: double startGameTime;
/// \brief Pointer to the current game state.
  private: State *currentState;
/// \brief Game half (1st half or 2nd half).
  private: Half half;
/// \brief Number of cycles of updates elapsed
  private: int cycleCounter;

    /*
    methods and constructors
    */
/// \brief Constructor.
  public: GameState();

/// \brief Destructor.
  public: virtual ~GameState();

/// \Clears the history of ball contacts
  public: void clearBallContactHistory();

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
/// the current one, the operation does not have any effect. New states have
/// their ball contact histories cleared.
/// \param [in] _newState
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

    /// \brief Check whether scoring conditions are met (another agent on same side that is not the kick off agent has touched the ball).
  public: void CheckCanScore();

/// \brief Set the agent's position only
  public: void MoveAgent(Agent &agent, const ignition::math::Vector3<double> &pos);

/// \brief Set the agent's position and yaw (beaming the agent)
  public: void MoveAgent(Agent &agent, const double x, const double y, const double yaw);

/// \brief Set the agent's position and yaw noisily (beaming the agent)
  public: void MoveAgentNoisy(Agent &agent, const double x, const double y, const double yaw);

/// \brief Set the agent's position and orientation
  public: void MoveAgent(Agent &agent, const ignition::math::Vector3<double> &pos, const ignition::math::Quaternion<double> &rot);

/// \brief Move agent to side of field, the side depends on the agent's side
  public: void MoveAgentToSide(Agent &agent);

/// \brief Check if the first half or the game ends, also update elapsed time.
  public: void CheckTiming();

/// \brief Check the ball's position looking for goals or out of bounds.
  public: void CheckBall();

/// \brief Checks that during goal kick, no members of opposing team are inside penalty area
  public: void CheckGoalKickIllegalDefense(Team::Side _teamAllowed);

/// \brief Check that no more than 3 players are in penalty area.
  public: void CheckIllegalDefense();

    /// \brief Check that no player has fallen or remain still for too long.
  public: void CheckImmobility();

/// \brief Helper method for CheckIllegalDefense()
  private: void CheckIllegalDefense_helper(Team *team);

/// \brief Check crowding rules.
  public: void CheckCrowding();
/// \brief Helper method for CheckCrowding()
  private: void CheckCrowding_helper(Team *team);

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
  public: void setBallVel(const ignition::math::Vector3<double> &_ballVel);

/// \brief Set the angular velocity of the ball
/// \param[in] Target angular velocity
  public: void setBallAngVel(const ignition::math::Vector3<double> &_ballAngVel);

/// \brief Method executed each time the game state changes.
  private: void Initialize();

/// \brief Add agent to game state
/// \param[in] Agent number
/// \param[in] Agent name
  public: bool addAgent(int uNum, std::string teamName);

/// \brief Remove agent from game state
/// \param[in] Agent number
/// \param[in] Agent name
  public: bool removeAgent(int uNum, std::string teamName);

/// \brief Beam the agent if the play mode allows it
  public: bool beamAgent(int uNum, std::string teamName, double x, double y, double rot);

    /*
    Getter and setter methods
    */

/// \brief Get the game's half.
/// \returns if the game is in the first half or if is in the second.
  public: Half GetHalf()
    {
      return half;
    };

/// \brief Set the game half.
/// \param[in] _newHalf 1 for first half or 2 for second half.
  public: void SetHalf(Half _newHalf)
    {
      half = _newHalf;
    };

/// \brief Get the elapsed time since beginning of half.
  public: double getElapsedGameTime()
    {
      return gameTime - startGameTime;
    }

/// \brief Get the elapsed time since last cycle.
  private: double getElapsedCycleGameTime()
    {
      return gameTime - prevCycleGameTime;
    }

/// \brief Get the game time.
  public: void setGameTime(double _gameTime)
    {
      gameTime = _gameTime;
    }

/// \brief Set the game time.
  public: double getGameTime()
    {
      return gameTime;
    }

/// \brief Get the game time when play starts
  public: double getStartGameTime()
    {
      return startGameTime;
    }
/// \brief Set the game time when play starts
  public: void setStartGameTime(double _startGameTime)
    {
      startGameTime = _startGameTime;
    }

/// \brief Set the cycle counter
  public: void setCycleCounter(int _cycleCounter)
    {
      cycleCounter = _cycleCounter;
      if (GameState::useCounterForGameTime) {
        double newGameTime = 0.02 * cycleCounter;
        double offsetTime = newGameTime - gameTime;
        prevCycleGameTime += offsetTime;
        gameTime = newGameTime;
      }
    }

/// \brief Get the cycle counter
  public: int getCycleCounter()
    {
      return cycleCounter;
    }

/// \brief Get the current state
  public: State *getCurrentState()
    {
      return currentState;
    }

/// \brief Get the team who last touched the ball.
  public: Team::Side *getLastSideTouchedBall()
    {
      if (getLastBallContact() != NULL) {
        return &(getLastBallContact()->side);
      } else {
        return NULL;
      }
    }

    /// \brief Get the player who last touched the ball.
  public: BallContact *getLastBallContact()
    {
      if (ballContactHistory.size() > 0) {
        return ballContactHistory.at(ballContactHistory.size() - 1).get();
      } else {
        return NULL;
      }
    }

// /// \brief Set the beamHeight
//   public: void setBeamHeight(double x)
//     {
//       beamHeight = x;
//     }

// /// \brief Get the beamHeight
//   public: double getBeamHeight()
//     {
//       return beamHeight;
//     }

// /// \brief Set the player limit
//   public: void setPlayerLimit(int x)
//     {
//       playerLimit = x;
//     }

// /// \brief Get the player limit
//   public: int getPlayerLimit()
//     {
//       return playerLimit;
//     }
//     /// \brief Set the players in penalty box limit
//   public: void setPenaltyBoxLimit(int x)
//     {
//       penaltyBoxLimit = x;
//     }

// /// \brief Get the players in penalty box limit
//   public: int getPenaltyBoxLimit()
//     {
//       return penaltyBoxLimit;
//     }

};

#endif
