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

#include "robocup3ds/GameState.hh"
#include "robocup3ds/Geometry.hh"

/// \class Perceptor Perceptor.hh robocup3ds/Perceptor.hh
/// \brief This class keeps up to date the information sent
/// to the agent
class Perceptor
{
  /// \Brief Constructor for Perceptor object
  /// \Param[in] Pointer to GameState object
  public: Perceptor(GameState *_gameState);

  /// \Brief Function to update all relevant agents perception info
  public: void Update();

  /// \Brief Helper function to update line info
  /// \Param[in] _line Line object
  /// \Param[out] _agent Agent object whose perception is updated
  private: void UpdateLine(GameState::Agent &_agent,
    const ignition::math::Line3<double> &_line) const;

  /// \Brief Function to update landmark info
  /// \Param[in] _landmarkname Name of landmark
  /// \Param[in] _landmark Position of landmark
  /// \Param[out] _agent Agent object whose perception is updated
  private: void UpdateLandmark(GameState::Agent &_agent,
                const std::string &_landmarkname,
                const ignition::math::Vector3<double> &_landmark) const;

  /// \Brief Noise added to all observations
  /// \Param[in] _pt Point object
  private: ignition::math::Vector3<double>
    addNoise(const ignition::math::Vector3<double> &_pt) const;

  /// \Brief A constant noise that is added to all observations
  private: static ignition::math::Vector3<double> fixedNoise;

  /// \Brief Sigma for dynamic noise that is added to all observations
  private: static ignition::math::Vector3<double> dNoiseSigma;

  /// \Brief Pointer to GameState object
  private: GameState *gameState;

  /// \Brief View frustrum
  /// We model view frustrum as a vector of four planes
  private: std::vector <ignition::math::Plane<double> > viewFrustrum;
};

#endif
