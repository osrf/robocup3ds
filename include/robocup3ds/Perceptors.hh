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

#ifndef _GAZEBO_ROBOCUP3DS_PERCEPTORS_HH_
#define _GAZEBO_ROBOCUP3DS_PERCEPTORS_HH_

#include <string>
#include <vector>
#include <ignition/math.hh>

#include "robocup3ds/Geometry.hh"

class Agent;
class GameState;
class RCPServer;

/// \brief This class keeps up to date the information sent
/// to the agent
class Perceptor
{
  /// \brief Constructor for Perceptor object
  /// \param[in] _gameState Pointer to GameState object
  public: Perceptor(GameState *const _gameState);

  /// \brief Destructor for Perceptor object
  public: ~Perceptor() = default;

  /// \brief Method used to set view frustum
  /// \param[in] _hfov Horizontal field of view in degrees
  /// \param[in] _vfov Vertical field of view in degrees
  public: void SetViewFrustum(const double _hfov, const double _vfov);

  /// \brief Method used to get view frustum
  /// \return A vector of planes of the view frustum
  public: const std::vector <ignition::math::Plane<double>>
                &GetViewFrustum() const;

  /// \brief Set the transformation matrix from global to local
  /// coordinates for an agent
  /// \param[in] _agent Agent object
  public: void SetG2LMat(const Agent &_agent);

  /// \brief Function to update all relevant agents perception info
  public: void Update();

  /// \brief Helper function to update line info
  /// \param[out] _agent Agent object whose perception is updated
  /// \param[in] _line Line object
  public: void UpdateLine(Agent &_agent,
    const ignition::math::Line3<double> &_line) const;

  /// \brief Function to update landmark info
  /// \param[out] _agent Agent object whose perception is updated
  /// \param[in] _landmarkname Name of landmark
  /// \param[in] _landmark Position of landmark
  public: void UpdateLandmark(Agent &_agent,
                const std::string &_landmarkname,
                const ignition::math::Vector3<double> &_landmark) const;

  /// \brief Function to update positions of other agents
  /// \param[out] _agent Agent whose perception we are updating
  /// \param[in] _otherAgent Other agent whose position is updated
  public: void UpdateOtherAgent(Agent &_agent,
    const Agent &_otherAgent) const;

  /// \brief Function to update message that agent hears
  /// \param[out] _agent Agent whose perception we are updating
  public: void UpdateAgentHear(Agent &_agent) const;

  /// \brief Function to convert perception information to s-expressions
  /// and write it to a string
  /// \param[in] _agent Agent whose perception we are updating
  /// \param[out] _string Buffer we are writing to
  /// \param[in] _size Size of the buffer
  /// \return True if buffer is large enough
  public: int Serialize(const Agent &_agent, char *_string,
                        const int _size) const;

  /// \brief Converts points to a string
  /// \param[in] _label Label for point
  /// \param[in] _pt Point object
  /// \param[in] _string Pointer to string buffer
  /// \param[in] _size Size of string buffer
  /// \return Number of characters written
  private: inline int SerializePoint(const char *_label,
                              const ignition::math::Vector3<double> &_pt,
                              char *_string, const int _size) const;

  /// \brief Function to add noise to all observations
  /// \param[in] _pt Point object
  /// \return A modified point object with noise
  private: ignition::math::Vector3<double>
    addNoise(const ignition::math::Vector3<double> &_pt) const;

  /// \brief Indicates whether current cycle requires updating perception and
  /// sending to server
  /// \return True if current cycle is right cycle to update perception
  private: bool UpdatePerception() const;

  /// \brief Indicates which side should speak this game cycle
  /// \return Side whose team can speak
  private: Team::Side SideToSpeak() const;

  /// \brief Indicates whether current cycle requires updating hearing info and
  /// sending to server
  /// \param[in] _team Team object
  /// \return True if current cycle is right cycle to update hearing
  private: bool UpdateHear(const Team &_team) const;

  /// \brief Frequency at which we update the visualization
  /// (every x gamestate cycles)
  public: static int updateVisualFreq;

  /// \brief Flag whether to add noise to observations or not
  public: static bool useNoise;

  /// \brief Distance of message where it still can be heard
  public: static const double kHearDist;

  /// \brief Multiplier for the distance noise
  public: static const double kDistNoiseScale;

  /// \brief Frequency at which we update the hearing for each side
  /// (every 2 gamestate cycles)
  public: static const int kUpdateHearFreq;

  /// \brief A constant noise that is added to all observations
  private: static const ignition::math::Vector3<double> kFixedNoise;

  /// \brief Sigma for dynamic noise that is added to all observations
  private: static const ignition::math::Vector3<double> kNoiseSigma;

  /// \brief Pointer to GameState object
  private: GameState *const gameState;

  /// \brief 4x4 transformation to go from global to local coordinates
  private: ignition::math::Matrix4<double> G2LMat;

  /// \brief View frustum
  /// We model view frustum as a vector of four planes
  private: std::vector <ignition::math::Plane<double>> viewFrustum;
};

#endif
