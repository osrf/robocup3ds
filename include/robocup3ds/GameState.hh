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

#include <boost/scoped_ptr.hpp>
#include <boost/thread/condition_variable.hpp>
#include <boost/thread/mutex.hpp>
#include <vector>
#include <string>
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
#include "robocup3ds/states/PlayState.hh"

class GameState
{
  private:

    /// \brief Internal Logger for GameState
    class Logger
    {
      private:
        GameState *gameState;

      public:
        int normalLevel, errorLevel;

        Logger(GameState *_gameState, int _normalLevel, int _errorLevel):
          gameState(_gameState),
          normalLevel(_normalLevel),
          errorLevel(_errorLevel)
        {}

        void log(std::string _message, int _level)
        {
          if (_level >= normalLevel)
          {
            std::cout << "[" << gameState->cycleCounter << "]["
                      << gameState->getGameTime() << "][lvl:" <<
                      _level << "]\t" << _message.c_str();
          }
        }

        void logErr(std::string _message, int _level)
        {
          if (_level >= errorLevel)
          {
            std::cout << "\033[31m[" << gameState->cycleCounter << "]["
                      << gameState->getGameTime() << "][lvl:" <<
                      _level << "]\t" << _message.c_str();
          }
        }
    };

  public:

    class Agent;

    /// \brief Team class.
    class Team
    {
      public:
        /// \brief Enum for the team side
        enum Side {NEITHER = -1, LEFT, RIGHT};
        /// \brief Name of the team.
        std::string name;
        /// \brief All the members in the team.
        std::vector<Agent> members;
        /// \brief Side of the team
        Side side;
        /// \brief Team score
        int score;
        /// \brief Number players in penalty area
        int numPlayersInPenaltyBox;
        /// \brief Can score goal or not
        bool canScore;

        /// \brief Constructor for Team object
        Team(std::string _name, Side _side, int _score, int _playerLimit):
          name(_name),
          side(_side),
          score(_score)
        {
          this->members.reserve(_playerLimit);
          this->numPlayersInPenaltyBox = 0;
          this->canScore = false;
        }
    };

    /// \brief Agent class.
    class Agent
    {
      public:
        /// \brief Enum for the agent status
        enum AgentStatus {RELEASED, STOPPED};
        /// \brief Agent unique id
        int uNum;
        //   /// \brief Agent name
        // public: std::string name;
        /// \brief Pointer to team that agent belongs to
        Team *team;
        /// \brief Agent status
        AgentStatus status;
        /// \brief Agent position
        ignition::math::Vector3<double> pos;
        /// \brief Agent position
        ignition::math::Vector3<double> prevPos;
        /// \brief Agent rotation
        ignition::math::Quaternion<double> rot;
        /// \brief Flag whether to update agent pose in world to match
        /// gamestate.
        bool updatePose;
        /// \brief Flag whether agent is in penalty box
        bool inPenaltyBox;
        /// \brief Stores time the agent has not moved
        double timeImmoblized;
        /// \brief Stores time the agent has fallen
        double timeFallen;
        /// \brief Flag whether player is goalkeeper
        bool isGoalKeeper()
        {
          return this->uNum == 1;
        }

        /// \brief Constructor for Agent object
        Agent(int _uNum, Team *_team):
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
    };

    /// \brief Class for storing ball contact information
    class BallContact
    {
      public:
        /// \brief Unum of agent who touched ball
        int uNum;
        /// \brief Side of agent who touched ball
        Team::Side side;
        /// \brief Time when agent stopped contacting the ball
        /// (if contact is longer than an instant)
        double contactTime;
        /// \brief Position where agent contacted ball
        ignition::math::Vector3<double> contactPos;
        /// \brief Pointer to team of agent who touched ball
        std::string teamName;

        /// \brief Constructor for Ball Contact object
        BallContact(int _uNum, Team::Side _side, double _contactTime,
                    ignition::math::Vector3<double> _contactPos):
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

    // various play mode state names
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

    /// \brief Duration of full game
    static double SecondsFullGame;
    /// \brief Duration of half a game
    static double SecondsEachHalf;
    /// \brief Duration of pause after scoring a goal
    static double SecondsGoalPause;
    /// \brief Duration of pause after entering kickIn, cornerKick, goalKick
    /// play mode
    static double SecondsKickInPause;
    /// \brief Duration of of kickIn, cornerKick, goalKick play modes
    static double SecondsKickIn;
    /// \brief Duration of before kick off play mode
    static double SecondsBeforeKickOff;
    /// \brief Duration of kick off play mode
    static double SecondsKickOff;
    /// \brief Radius where not allowed team cannot approach ball
    static double dropBallRadius;
    /// \brief Every cycle counts as 20ms of game time (for `fake simulations)
    static bool useCounterForGameTime;
    /// \brief Max players per team
    static int playerLimit;
    /// \brief Max players in penalty box
    static int penaltyBoxLimit;
    /// \brief Beam height
    static double beamHeight;
    /// \brief Distance when to enable crowding rules
    static double crowdingEnableRadius;
    /// \brief Distance when to reposition one of two players
    static double innerCrowdingRadius;
    /// \brief Distance when to reposition one of three players
    static double outerCrowdingRadius;
    /// \brief Timeout when player remains immobile too long
    static double immobilityTimeLimit;
    /// \brief Timeout when player remains fallen too long
    static double fallenTimeLimit;

// map containing configuration variables
    std::map<std::string, std::string> *config;
/// \brief Enum for which half it is
    enum Half {FIRST_HALF, SECOND_HALF};
/// \brief Whether currentState has changed in the current update cycle or not
    bool hasCurrentStateChanged;
/// \brief History of ball contacts;
    std::vector<boost::shared_ptr<BallContact> > ballContactHistory;
/// \brief Pointer to the ball contact that causes the game to
/// transition from kick off to play on
    BallContact *touchBallKickoff;
/// \brief All the teams.
    std::vector<Team *> teams;
/// \brief Position of soccer ball.
    bool updateBallPose;
/// \brief Flag whether to update ball position in world to match game state.

  private:
    ignition::math::Vector3<double> ballPos;
/// \brief Angular velocity of soccer ball.
    ignition::math::Vector3<double> ballAngVel;
/// \brief Linear velocity of soccer ball.
    ignition::math::Vector3<double> ballVel;
/// \brief Game time.
    double gameTime;
/// \brief Game time during previous cycle.
    double prevCycleGameTime;
/// \brief Time when half starts (elapsed game time is essentially
/// gameTime - startGameTime)
    double startGameTime;
/// \brief Pointer to the current game state.
    State *currentState;
/// \brief Game half (1st half or 2nd half).
    Half half;
/// \brief Number of cycles of updates elapsed
    int cycleCounter;

    /*
    methods and constructors
    */
  public:

/// \brief Constructor.
    GameState();

/// \brief Destructor.
    virtual ~GameState();

/// \brief Function for loading gameState configuration variables
    void loadConfiguration();

/// \Clears the history of ball contacts
    void clearBallContactHistory();

/// \brief Update the robocup simulation state.
/// \param[in] _info Information used in the update event.
    void Update();

/// \brief During some of the states of the game the players are not allowed
/// to move (for example during kickoff). This method releases the players
/// when it's time to move the robots (for example in play mode).
    void ReleasePlayers();

/// \brief During some of the states of the game the players are not allowed
/// to move (for example during kickoff). This method stops the players by
/// creating a joint to the world for each player.
    void StopPlayers();

/// \brief Set the current game state. If the new state is the same than
/// the current one, the operation does not have any effect. New states have
/// their ball contact histories cleared.
/// \param [in] _newState
    void SetCurrent(State *_newState);

/// \brief Drops the ball at its current position and move all players away
/// by the free kick radius. If the ball is off the field, it is brought
/// back within bounds.
/// \param[in] Team::LEFT if the left team is allowed to be close to the
/// ball. Team::RIGHT if the right team is allowed or any other number if
/// none of the teams are allowed to be within the free kick radius.
    void DropBallImpl(const Team::Side _teamAllowed);

/// \brief Checks whether the double touching rules are violated
    void CheckDoubleTouch();

/// \brief Check whether scoring conditions are met (another agent on
/// same side that is not the kick off agent has touched the ball).
    void CheckCanScore();

/// \brief Check that agents stay on their own side during kick offs
    void CheckOffSidesOnKickOff(Team::Side _kickingSide);

/// \brief Set the agent's position only
    void MoveAgent(Agent &_agent,
                   const ignition::math::Vector3<double> &_pos);

/// \brief Set the agent's position and yaw (beaming the agent)
    void MoveAgent(Agent &_agent, const double _x, const double _y,
                   const double yaw);

/// \brief Set the agent's position and yaw noisily (beaming the agent)
    void MoveAgentNoisy(Agent &_agent, const double _x, const double _y,
                        const double _yaw);

/// \brief Set the agent's position and orientation
    void MoveAgent(Agent &_agent, const ignition::math::Vector3<double> &_pos,
                   const ignition::math::Quaternion<double> &_rot);

/// \brief Move agent to side of field, the side depends on the agent's side
    void MoveAgentToSide(Agent &_agent);

/// \brief Move agent back to their own side if they are offsides during kick
/// offs
    void MoveOffSideAgent(Agent &_agent);

/// \brief Check if the first half or the game ends, also update elapsed time.
    void CheckTiming();

/// \brief Check the ball's position looking for goals or out of bounds.
    void CheckBall();

/// \brief Checks that during goal kick, no members of opposing team
/// are inside penalty area
    void CheckGoalKickIllegalDefense(Team::Side _teamAllowed);

/// \brief Check that no more than 3 players are in penalty area.
    void CheckIllegalDefense();

    /// \brief Check that no player has fallen or remain still for too long.
    void CheckImmobility();

/// \brief Check crowding rules.
    void CheckCrowding();

/// \brief Get the ball position in the field.
/// \return The position of the ball.
    ignition::math::Vector3<double> GetBall();

/// \brief Move the ball to the center of field.
    void MoveBallToCenter();

/// \brief Move ball to goal kick position.
    void MoveBallForGoalKick();

/// \brief Move the ball to the corner of the field.
    void MoveBallToCorner();

/// \brief Move the ball in bounds if it is out of bounds.
    void MoveBallInBounds();

/// \brief Move the ball to a given position.
/// \param[in] Target position.
    void MoveBall(const ignition::math::Vector3<double> &_ballPos);

/// \brief Set the linear velocity of the ball
/// \param[in] Target linear velocity
    void setBallVel(const ignition::math::Vector3<double> &_ballVel);

/// \brief Set the angular velocity of the ball
/// \param[in] Target angular velocity
    void setBallAngVel(const ignition::math::Vector3<double> &_ballAngVel);

/// \brief Add agent to game state
/// \param[in] Agent number
/// \param[in] Agent name
    bool addAgent(int _uNum, std::string _teamName);

/// \brief Remove agent from game state
/// \param[in] Agent number
/// \param[in] Agent name
    bool removeAgent(int _uNum, std::string _teamName);

/// \brief Beam the agent if the play mode allows it
    bool beamAgent(int _uNum, std::string teamName, double _x, double _y,
                   double _rot);


  private:

/// \brief Method executed each time the game state changes.
    void Initialize();

/// \brief Helper function for loading gameState configuration variables
    bool loadConfigParameter(const std::string &_key, double &_value);

/// \brief Helper function for loading gameState configuration variabless
    bool loadConfigParameterBool(const std::string &_key, bool &_boolValue);

    /*
    Getter and setter methods
    */
  public:

/// \brief Get the game's half.
/// \returns if the game is in the first half or if is in the second.
    Half GetHalf()
    {
      return this->half;
    }

/// \brief Set the game half.
/// \param[in] _newHalf 1 for first half or 2 for second half.
    void SetHalf(Half _newHalf)
    {
      this->half = _newHalf;
    }

/// \brief Get the elapsed time since beginning of half.
    double getElapsedGameTime()
    {
      return this->gameTime - this->startGameTime;
    }

/// \brief Get the elapsed time since last cycle.
    double getElapsedCycleGameTime()
    {
      return this->gameTime - this->prevCycleGameTime;
    }

/// \brief Get the game time.
    void setGameTime(double _gameTime)
    {
      this->gameTime = _gameTime;
    }

/// \brief Set the game time.
    double getGameTime()
    {
      return this->gameTime;
    }

/// \brief Get the game time when play starts
    double getStartGameTime()
    {
      return this->startGameTime;
    }
/// \brief Set the game time when play starts
    void setStartGameTime(double _startGameTime)
    {
      this->startGameTime = _startGameTime;
    }

/// \brief Set the cycle counter
    void setCycleCounter(int _cycleCounter)
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
    int getCycleCounter()
    {
      return this->cycleCounter;
    }

/// \brief Get the current state
    State *getCurrentState()
    {
      return this->currentState;
    }

/// \brief Get the team who last touched the ball.
    Team::Side *getLastSideTouchedBall()
    {
      if (this->getLastBallContact() != NULL)
      {
        return &(this->getLastBallContact()->side);
      }
      else
      {
        return NULL;
      }
    }

    /// \brief Get the player who last touched the ball.
    BallContact *getLastBallContact()
    {
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
