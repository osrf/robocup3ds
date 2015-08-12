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
#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <sstream>
#include <string>
#include <vector>

#include "Robocup3dsGUIPlugin.hh"

using namespace gazebo;

// Register this plugin with the simulator
GZ_REGISTER_GUI_PLUGIN(Robocup3dsGUIPlugin)

/////////////////////////////////////////////////
Robocup3dsGUIPlugin::Robocup3dsGUIPlugin()
  : GUIPlugin()
{
  // Set the frame background and foreground colors
  this->setStyleSheet(
    "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  this->AddSimTimeWidget(frameLayout);
  this->AddGameStateWidget(frameLayout);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(50, 10);
  this->resize(1000, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->statsSub = this->node->Subscribe("~/world_stats",
                                         &Robocup3dsGUIPlugin::OnStats, this);
  this->gameSub = this->node->Subscribe("~/robocup3ds/state",
                                        &Robocup3dsGUIPlugin::OnGameState,
                                        this);
  gzmsg << "Robocup GUI plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddSimTimeWidget(QHBoxLayout *_frameLayout)
{
  QLabel *label = new QLabel(tr("Sim Time:"));

  // Create a time label
  QLabel *timeLabel = new QLabel();

  // Add the label to the frame's layout
  _frameLayout->addWidget(label);
  _frameLayout->addWidget(timeLabel);
  connect(this, SIGNAL(SetSimTime(QString)),
          timeLabel, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddGameStateWidget(QHBoxLayout *_frameLayout)
{
  QLabel *label = new QLabel(tr("Game Time:"));
  QLabel *gameTimeLabel = new QLabel();
  _frameLayout->addWidget(label);
  _frameLayout->addWidget(gameTimeLabel);
  connect(this, SIGNAL(SetGameTime(QString)),
          gameTimeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  QLabel *teamLabel = new QLabel();
  _frameLayout->addWidget(teamLabel);
  connect(this, SIGNAL(SetTeam(QString)),
          teamLabel, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
Robocup3dsGUIPlugin::~Robocup3dsGUIPlugin()
{
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::OnGameState(ConstGzStringPtr &_msg)
{
  std::ostringstream stream;
  std::string rawString = _msg->data();

  stream.str("");
  size_t i = rawString.find(":");
  std::string gameTime = rawString.substr(0, i);
  rawString = rawString.substr(i+1);

  double _gameTime = std::stod(gameTime);
  int sec = static_cast<double>(_gameTime);
  int msec = rint((_gameTime - sec) * 1e3);
  stream << std::setw(4) << std::setfill('0') << sec << ".";
  stream << std::setw(2) << std::setfill('0') << msec;
  this->SetGameTime(QString::fromStdString(stream.str()));

  stream.str("");
  this->SetTeam(QString::fromStdString(rawString));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::OnStats(ConstWorldStatisticsPtr &_msg)
{
  this->SetSimTime(QString::fromStdString(
                     this->FormatTime(_msg->sim_time())));
}

/////////////////////////////////////////////////
std::string Robocup3dsGUIPlugin::FormatTime(const msgs::Time &_msg) const
{
  std::ostringstream stream;
  // unsigned int day, hour, min, sec, msec;
  stream.str("");
  // sec = _msg.sec();
  const auto msec = rint(_msg.nsec() * 1e-6);
  stream << std::setw(4) << std::setfill('0') << _msg.sec() << ".";
  stream << std::setw(2) << std::setfill('0') << msec;

  // day = sec / 86400;
  // sec -= day * 86400;
  // hour = sec / 3600;
  // sec -= hour * 3600;
  // min = sec / 60;
  // sec -= min * 60;
  // msec = rint(_msg.nsec() * 1e-6);

  // stream << std::setw(2) << std::setfill('0') << day << " ";
  // stream << std::setw(2) << std::setfill('0') << hour << ":";
  // stream << std::setw(2) << std::setfill('0') << min << ":";
  // stream << std::setw(2) << std::setfill('0') << sec << ".";
  // stream << std::setw(3) << std::setfill('0') << msec;

  return stream.str();
}
