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

#ifndef _GAZEBO_ROBOCUP3DS_PLUGIN_HH_
#define _GAZEBO_ROBOCUP3DS_PLUGIN_HH_

#include <gazebo/gazebo.hh>

namespace gazebo
{
  class Robocup3dsPlugin : public WorldPlugin
  {
    /// \brief Constructor.
    public: Robocup3dsPlugin();

    /// \brief Destructor.
    public: virtual ~Robocup3dsPlugin();

    // Documentation inherited.
    public: virtual void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf);

    // Documentation inherited.
    public: virtual void Init();

    /// \brief Update the robocup simulation state.
    /// \param[in] _info Information used in the update event.
    public: void Update(const common::UpdateInfo &_info);

    /// \brief Pointer to the update event connection.
    private: event::ConnectionPtr updateConnection;
  };
}
#endif
