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

#ifndef _GAZEBO_PERCEPTORS_HH_
#define _GAZEBO_PERCEPTORS_HH_

#include <ignition/math.hh>
#include <map>
#include <memory>
#include <string>
#include <vector>

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
  public: Perceptor(GameState *const _gameState, RCPServer *const _server);

  /// \brief Destructor for Perceptor object
  public: ~Perceptor();

  /// \brief Method used to set view frustum based on HFov, VFov
  public: void SetViewFrustum();

  /// \brief Method used to get view frustum
  public: std::vector <ignition::math::Plane<double>> &GetViewFrustum();

  /// \brief Function to update all relevant agents perception info
  public: void Update();

  /// \brief Helper function to update line info
  /// \param[in] _agent Agent object whose perception is updated
  /// \param[in] _line Line object
  public: void UpdateLine(Agent &_agent,
    const ignition::math::Line3<double> &_line) const;

  /// \brief Function to update landmark info
  /// \param[in] _agent Agent object whose perception is updated
  /// \param[in] _landmarkname Name of landmark
  /// \param[in] _landmark Position of landmark
  public: void UpdateLandmark(Agent &_agent,
                const std::string &_landmarkname,
                const ignition::math::Vector3<double> &_landmark) const;

  /// \brief Function to update positions of other agents
  /// \param[in] _agent Agent whose perception we are updating
  /// \param[in] _otherAgent Other agent whose position is updated
  public: void UpdateOtherAgent(Agent &_agent,
    const Agent &_otherAgent) const;

  /// \brief Function to update message that agent hears
  /// \param[in] _agent Agent whose perception we are updating
  public: void UpdateAgentHear(Agent &_agent) const;

  /// \brief Function to send messages to server
  public: void SendToServer() const;

  /// \brief Function to convert perception information to s-expressions
  /// and write it to a string
  /// \param[in] _agent Agent whose perception we are updating
  /// \param[in] _string Buffer we are writing to
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

  /// \brief Set the transformation matrix from global to local
  /// coordinates for an agent
  /// \param[in] _agent Agent object
  public: void SetG2LMat(const Agent &_agent);

  /// \Brief Size of buffer used to store message sent to server
  public: static const int bufferSize;

  /// \brief Distance of message where it still can be heard
  public: static const double kHearDist;

  /// \brief Flag whether to add noise to observations or not
  public: static bool useNoise;

  /// \brief A constant noise that is added to all observations
  private: static ignition::math::Vector3<double> fixedNoise;

  /// \brief Sigma for dynamic noise that is added to all observations
  private: static ignition::math::Vector3<double> dNoiseSigma;

  /// \brief Pointer to GameState object
  private: GameState *const gameState;

  /// \brief Pointer to Server object
  private: RCPServer *const server;

  /// \brief Buffer used to store data sent to server
  private: char *buffer;

  /// \brief 4x4 transformation to go from global to local coordinates
  private: ignition::math::Matrix4<double> G2LMat;

  /// \brief View frustum
  /// We model view frustum as a vector of four planes
  private: std::vector <ignition::math::Plane<double> > viewFrustum;
};

#endif
