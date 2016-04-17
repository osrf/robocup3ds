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

#include <gazebo/common/Time.hh>
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
  this->time = common::Time();

  // Set the frame background and foreground colors
  this->setStyleSheet(
    "QFrame { background-color : rgba(100, 100, 100, 255); color : white; }");

  // Create the main layout
  QHBoxLayout *mainLayout = new QHBoxLayout;

  // Create the frame to hold all the widgets
  QFrame *mainFrame = new QFrame();

  // Create the layout that sits inside the frame
  QHBoxLayout *frameLayout = new QHBoxLayout();

  this->AddGameTimeWidget(frameLayout);
  this->AddPlaymodeWidget(frameLayout);
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
  this->resize(800, 30);

  // Create a node for transportation
  this->node = transport::NodePtr(new transport::Node());
  this->node->Init("default");
  this->gameSub = this->node->Subscribe("~/robocup3ds/state",
                                        &Robocup3dsGUIPlugin::OnGameState,
                                        this);
  this->playmodePub =
    this->node->Advertise<msgs::GzString>("~/robocup3dsGUI/playmode");

  gzmsg << "Robocup GUI plugin loaded" << std::endl;
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddGameTimeWidget(QHBoxLayout *_frameLayout)
{
  QLabel *label = new QLabel(tr("Time:"));
  QLabel *gameTimeLabel = new QLabel();
  QFont myFont;
  QFontMetrics fm(myFont);
  QString str("00:00:000000");
  gameTimeLabel->setFixedWidth(fm.width(str));
  _frameLayout->addWidget(label);
  _frameLayout->addWidget(gameTimeLabel);
  connect(this, SIGNAL(SetGameTime(QString)),
          gameTimeLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  _frameLayout->addSpacerItem(new QSpacerItem(10, 1,
    QSizePolicy::MinimumExpanding));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddTeamWidget(QHBoxLayout *_frameLayout)
{
  _frameLayout->addSpacerItem(new QSpacerItem(30, 1, QSizePolicy::Fixed));

  QLabel *teamLabel = new QLabel();
  teamLabel->setStyleSheet("QLabel {color : #99FFFF;}");
  _frameLayout->addWidget(teamLabel);
  connect(this, SIGNAL(SetLeftTeam(QString)),
          teamLabel, SLOT(setText(QString)), Qt::QueuedConnection);

  _frameLayout->addSpacerItem(new QSpacerItem(30, 1, QSizePolicy::Fixed));

  QLabel *teamLabel2 = new QLabel();
  teamLabel2->setStyleSheet("QLabel {color : #FFCCFF;}");
  _frameLayout->addWidget(teamLabel2);
  connect(this, SIGNAL(SetRightTeam(QString)),
          teamLabel2, SLOT(setText(QString)), Qt::QueuedConnection);
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::AddPlaymodeWidget(QHBoxLayout *_frameLayout)
{
  this->playmodeComboBox = new QComboBox(this);
  this->playmodeComboBox->addItem("BeforeKickOff");
  this->playmodeComboBox->addItem("KickOffLeft");
  this->playmodeComboBox->addItem("KickOffRight");
  this->playmodeComboBox->addItem("PlayOn");
  this->playmodeComboBox->addItem("KickInLeft");
  this->playmodeComboBox->addItem("KickInRight");
  this->playmodeComboBox->addItem("CornerKickLeft");
  this->playmodeComboBox->addItem("CornerKickRight");
  this->playmodeComboBox->addItem("GoalKickLeft");
  this->playmodeComboBox->addItem("GoalKickRight");
  this->playmodeComboBox->addItem("GameOver");
  this->playmodeComboBox->addItem("GoalLeft");
  this->playmodeComboBox->addItem("GoalRight");
  this->playmodeComboBox->addItem("FreeKickLeft");
  this->playmodeComboBox->addItem("FreeKickRight");
  QFont myFont;
  QFontMetrics fm(myFont);
  QString str("##########################");
  this->playmodeComboBox->view()->setFixedWidth(fm.width(str));

  _frameLayout->addWidget(this->playmodeComboBox);
  connect(this->playmodeComboBox, SIGNAL(currentIndexChanged(QString)),
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
  QString playmode = QString::fromStdString(gameState.substr(j + 1));
  int index = this->playmodeComboBox->findText(playmode);
  if (index != -1)
  {
    this->playmodeComboBox->blockSignals(true);
    this->playmodeComboBox->setCurrentIndex(index);
    this->playmodeComboBox->blockSignals(false);
  }

  this->time.Set(_gameTime);
  this->SetGameTime(QString::fromStdString(this->time.FormattedString(
                      common::Time::MINUTES)));

  i = rawString.find("$");
  this->SetLeftTeam(QString::fromStdString(rawString.substr(0, i)));
  this->SetRightTeam(QString::fromStdString(rawString.substr(i + 1)));
}

/////////////////////////////////////////////////
void Robocup3dsGUIPlugin::HandleSelectionChanged(const QString &_text)
{
  msgs::GzString msg;
  msg.set_data(_text.toStdString());
  this->playmodePub->Publish(msg);
}

