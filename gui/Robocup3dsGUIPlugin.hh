/*
 * Copyright (C) 2014-2015 Open Source Robotics Foundation
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
#ifndef _GAZEBO_ROBOCUP3DS_GUI_PLUGIN_HH_
#define _GAZEBO_ROBOCUP3DS_GUI_PLUGIN_HH_

#include <string>
#include <gazebo/common/Plugin.hh>
#include <gazebo/gui/GuiPlugin.hh>
#ifndef Q_MOC_RUN  // See: https://bugreports.qt-project.org/browse/QTBUG-22829
# include <gazebo/transport/transport.hh>
# include <gazebo/gui/gui.hh>
#endif

namespace gazebo
{
  class GAZEBO_VISIBLE Robocup3dsGUIPlugin : public GUIPlugin
  {
    Q_OBJECT

    /// \brief Constructor
    public: Robocup3dsGUIPlugin();

    /// \brief Destructor
    public: virtual ~Robocup3dsGUIPlugin();

    /// \brief A signal used to set the sim time line edit.
    /// \param[in] _string String representation of sim time.
    signals: void SetSimTime(QString _string);

    /// \brief A signal used to set the game state line edit.
    /// \param[in] _string String representation of game state.
    signals: void SetGameTime(QString _string);

    /// \brief A signal used to set the game state line edit.
    /// \param[in] _string String representation of game state.
    signals: void SetTeam(QString _string);

    /// \brief A signal used to set the game state line edit.
    /// \param[in] _string String representation of game state.
    signals: void SetTeam2(QString _string);

    /// \brief A signal used to set the game state line edit.
    /// \param[in] _string String representation of game state.
    signals: void SetPlaymode(QString _string);

    /// \brief Callback that received world statistics messages.
    /// \param[in] _msg World statistics message that is received.
    protected: void OnStats(ConstWorldStatisticsPtr &_msg);

    /// \brief Callback that received game state messages.
    /// \param[in] _msg Game state message that is received.
    protected: void OnGameState(ConstGzStringPtr &_msg);

    /// \brief Helper method to add a simulation time widget
    /// \param[in] _frameLayout Pointer to frame layout object
    protected: void AddSimTimeWidget(QHBoxLayout *_frameLayout);

    /// \brief Helper method to add a game state widget
    /// \param[in] _frameLayout Pointer to frame layout object
    protected: void AddGameStateWidget(QHBoxLayout *_frameLayout);

    /// \brief Helper function to format time string.
    /// \param[in] _msg Time message.
    /// \return Time formatted as a string.
    private: std::string FormatTime(const msgs::Time &_msg) const;

    /// \brief Node used to establish communication with gzserver.
    private: transport::NodePtr node;

    /// \brief Subscriber to world statistics messages.
    private: transport::SubscriberPtr statsSub;

    /// \brief Subscriber to game state messages.
    private: transport::SubscriberPtr gameSub;
  };
}

#endif
