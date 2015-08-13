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
  this->AddComboBox(frameLayout);
  this->AddTeamWidget(frameLayout);

  // Add frameLayout to the frame
  mainFrame->setLayout(frameLayout);

  // Add the frame to the main layout
  mainLayout->addWidget(mainFrame);

  // Remove margins to reduce space
  frameLayout->setContentsMargins(4, 4, 4, 4);
  mainLayout->setContentsMargins(0, 0, 0, 0);

  this->setLayout(mainLayout);

  // Position and resize this widget
  this->move(12, 12);
  this->resize(1300, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->statsSub = this->node->Subscribe("~/world_stats",
                                         &Robocup3dsGUIPlugin::OnStats, this);
  this->gameSub = this->node->Subscribe("~/robocup3ds/state",
                                        &Robocup3dsGUIPlugin::OnGameState,
                                        this);
  this->playmodePub =
    this->node->Advertise<msgs::GzString>("~/robocup3dsGUI/playmode");

  gzmsg << "Robocup GUI plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddSimTimeWidget(QHBoxLayout *_frameLayout)
{
  QLabel *label = new QLabel(tr("Sim Time:"));
  QLabel *timeLabel = new QLabel();
  QFont myFont;
  QFontMetrics fm(myFont);
  QString str("0000.00");
  timeLabel->setFixedWidth(fm.width(str));

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
  QFont myFont;
  QFontMetrics fm(myFont);
  QString str("0000.00");
  gameTimeLabel->setFixedWidth(fm.width(str));
  _frameLayout->addWidget(label);
  _frameLayout->addWidget(gameTimeLabel);
  connect(this, SIGNAL(SetGameTime(QString)),
          gameTimeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  QLabel *label2 = new QLabel(tr("Playmode:"));
  QLabel *playmodeLabel = new QLabel();
  _frameLayout->addWidget(label2);
  _frameLayout->addWidget(playmodeLabel);
  connect(this, SIGNAL(SetPlaymode(QString)),
          playmodeLabel, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddTeamWidget(QHBoxLayout *_frameLayout)
{
  _frameLayout->addSpacerItem(new QSpacerItem(0, 1, QSizePolicy::Expanding));

  QLabel *teamLabel = new QLabel();
  _frameLayout->addWidget(teamLabel);
  connect(this, SIGNAL(SetTeam(QString)),
          teamLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  _frameLayout->addSpacerItem(new QSpacerItem(30, 1, QSizePolicy::Fixed));

  QLabel *teamLabel2 = new QLabel();
  _frameLayout->addWidget(teamLabel2);
  connect(this, SIGNAL(SetTeam2(QString)),
          teamLabel2, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddComboBox(QHBoxLayout *_frameLayout)
{
  QLabel *label = new QLabel(tr("Select: "));
  QComboBox *comboBox = new QComboBox(this);
  comboBox->addItem("BeforeKickOff");
  comboBox->addItem("KickOffLeft");
  comboBox->addItem("KickOffRight");
  comboBox->addItem("PlayOn");
  comboBox->addItem("KickInLeft");
  comboBox->addItem("KickInRight");
  comboBox->addItem("CornerKickLeft");
  comboBox->addItem("CornerKickRight");
  comboBox->addItem("GoalKickLeft");
  comboBox->addItem("GoalKickRight");
  comboBox->addItem("GameOver");
  comboBox->addItem("GoalLeft");
  comboBox->addItem("GoalRight");
  comboBox->addItem("FreeKickLeft");
  comboBox->addItem("FreeKickRight");
  comboBox->setSizeAdjustPolicy(QComboBox::AdjustToContents);
  QFont myFont;
  QFontMetrics fm(myFont);
  QString str("#################");
  comboBox->view()->setFixedWidth(fm.width(str));

  _frameLayout->addWidget(label);
  _frameLayout->addWidget(comboBox);
  connect(comboBox, SIGNAL(currentIndexChanged(QString)),
          this, SLOT(HandleSelectionChanged(QString)));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::OnGameState(ConstGzStringPtr &_msg)
{
  std::ostringstream stream;
  std::string rawString = _msg->data();

  stream.str("");
  size_t i = rawString.find("$");
  std::string gameState = rawString.substr(0, i);
  rawString = rawString.substr(i + 1);

  size_t j = gameState.find(" ");
  double _gameTime = std::stod(gameState.substr(0, j));
  this->SetPlaymode(QString::fromStdString(gameState.substr(j + 1)));

  int sec = static_cast<double>(_gameTime);
  int msec = rint((_gameTime - sec) * 1e2);
  stream << std::setw(4) << std::setfill('0') << sec << ".";
  stream << std::setw(2) << std::setfill('0') << msec;
  this->SetGameTime(QString::fromStdString(stream.str()));

  i = rawString.find("$");
  this->SetTeam(QString::fromStdString(rawString.substr(0, i)));
  this->SetTeam2(QString::fromStdString(rawString.substr(i + 1)));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::OnStats(ConstWorldStatisticsPtr &_msg)
{
  std::ostringstream stream;
  stream.str("");
  const msgs::Time msg = _msg->sim_time();
  const auto msec = rint(msg.nsec() * 1e-7);
  stream << std::setw(4) << std::setfill('0') << msg.sec() << ".";
  stream << std::setw(2) << std::setfill('0') << msec;
  this->SetSimTime(QString::fromStdString(stream.str()));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::HandleSelectionChanged(const QString &_text)
{
  msgs::GzString msg;
  msg.set_data(_text.toStdString());
  this->playmodePub->Publish(msg);
}

