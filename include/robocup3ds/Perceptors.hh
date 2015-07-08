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

/// \brief This class keeps up to date the information sent
/// to the agent
class Perceptor
{
  /// \Brief Constructor for Perceptor object
  /// \Param[in] _gameState Pointer to GameState object
  public: Perceptor(GameState *const _gameState);

  /// \Brief Destructor for Perceptor object
  public: ~Perceptor();

  /// \Brief Method used to set view frustum based on HFov, VFov
  public: void SetViewFrustum();

  /// \Brief Method used to get view frustum
  public: std::vector <ignition::math::Plane<double>> &GetViewFrustum();

  /// \Brief Function to update all relevant agents perception info
  public: void Update();

  /// \Brief Helper function to update line info
  /// \Param[in] _agent Agent object whose perception is updated
  /// \Param[in] _line Line object
  public: void UpdateLine(Agent &_agent,
    const ignition::math::Line3<double> &_line) const;

  /// \Brief Function to update landmark info
  /// \Param[in] _agent Agent object whose perception is updated
  /// \Param[in] _landmarkname Name of landmark
  /// \Param[in] _landmark Position of landmark
  public: void UpdateLandmark(Agent &_agent,
                const std::string &_landmarkname,
                const ignition::math::Vector3<double> &_landmark) const;

  /// \Brief Function to update positions of other agents
  /// \Param[in] _agent Agent whose perception we are updating
  /// \Param[in] _otherAgent Other agent whose position is updated
  public: void UpdateOtherAgent(Agent &_agent,
    const Agent &_otherAgent) const;

  /// \Brief Function to update message that agent hears
  /// \Param[in] _agent Agent whose perception we are updating
  public: void UpdateAgentHear(Agent &_agent) const;

  /// \Brief Function to add noise to all observations
  /// \Param[in] _pt Point object
  /// \return A modified point object with noise
  private: ignition::math::Vector3<double>
    addNoise(const ignition::math::Vector3<double> &_pt) const;

  /// \Brief Set the transformation matrix from global to local
  /// coordinates for an agent
  /// \Param[in] _agent Agent object
  public: void SetG2LMat(const Agent &_agent);

  /// \Brief Flag whether to add noise to observations or not
  public: static bool useNoise;

  /// \Brief Distance of message where it still can be heard
  public: static const double kHearDist;

  /// \Brief A constant noise that is added to all observations
  private: static ignition::math::Vector3<double> fixedNoise;

  /// \Brief Sigma for dynamic noise that is added to all observations
  private: static ignition::math::Vector3<double> dNoiseSigma;

  /// \Brief Pointer to GameState object
  private: GameState *const gameState;

  /// \Brief 4x4 transformation to go from global to local coordinates
  private: ignition::math::Matrix4<double> G2LMat;

  /// \Brief View frustum
  /// We model view frustum as a vector of four planes
  private: std::vector <ignition::math::Plane<double> > viewFrustum;
};

#endif
