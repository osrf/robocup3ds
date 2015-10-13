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
#include <memory>
#include <string>
#include <utility>
#include "gtest/gtest.h"

#include "robocup3ds/Agent.hh"
#include "robocup3ds/Nao.hh"

/// \brief Test that methods of Team class work
TEST(AgentTest, TeamMethodsTest)
{
  EXPECT_EQ("left", Team::GetSideAsString(Team::Side::LEFT));
  EXPECT_EQ("right", Team::GetSideAsString(Team::Side::RIGHT));
  EXPECT_EQ("neither", Team::GetSideAsString(Team::Side::NEITHER));

  EXPECT_EQ(Team::Side::LEFT, Team::GetSideAsEnum("left"));
  EXPECT_EQ(Team::Side::RIGHT, Team::GetSideAsEnum("right"));
  EXPECT_EQ(Team::Side::NEITHER, Team::GetSideAsEnum("none"));
  EXPECT_EQ(Team::Side::NEITHER, Team::GetSideAsEnum("neither"));

  Team t1("red", Team::Side::LEFT, 0, 11);
  Team t2("blue", Team::Side::RIGHT, 0, 11);
  EXPECT_EQ(t1, t1);
  EXPECT_EQ(t2, t2);
  EXPECT_NE(t1, t2);
}

/// \brief Test that methods of Team class work
TEST(AgentTest, AgentMethodsTest)
{
  std::shared_ptr<Team> t1 =
    std::make_shared<Team>("red", Team::Side::LEFT, 0, 11);
  Agent a1(1, t1);
  Agent a2(2, t1);
  EXPECT_EQ(a1, a1);
  EXPECT_EQ(a2, a2);
  EXPECT_NE(a2, a1);

  EXPECT_EQ("1_red", a1.GetName());
  AgentId agentId(1, "red");
  EXPECT_EQ(agentId, a1.GetAgentID());

  EXPECT_EQ("1_red", Agent::GetName(1, "red"));
  int unum;
  std::string name;
  EXPECT_TRUE(Agent::CheckAgentName("1_red", unum, name));
  EXPECT_EQ(unum, 1);
  EXPECT_EQ(name, "red");
  EXPECT_FALSE(Agent::CheckAgentName("bird", unum, name));

  EXPECT_TRUE(a1.IsGoalKeeper());
  EXPECT_FALSE(a2.IsGoalKeeper());

  AgentSay as;
  EXPECT_FALSE(as.isValid);
  AgentHear ah;
  EXPECT_FALSE(ah.isValid);

  Agent a3(4, nullptr);
  EXPECT_EQ(a3.GetName(), "4");
  AgentId agentId2(4, "");
  EXPECT_EQ(agentId2, a3.GetAgentID());
}

/// \brief Test that body type parameters are correct
TEST(AgentTest, BodyTypeTest)
{
  std::shared_ptr<Team> t1 =
    std::make_shared<Team>("red", Team::Side::LEFT, 0, 11);
  Agent a1(1, t1);

  EXPECT_DOUBLE_EQ(a1.bodyType->Height(), 0.6);
  EXPECT_DOUBLE_EQ(a1.bodyType->TorsoHeight(), 0.3);
  EXPECT_EQ(a1.bodyType->HingeJointEffectorMap().at("lae3"), "LElbowYaw");
  EXPECT_EQ(a1.bodyType->DefaultModelName(), "naoH25V40");
}

int main(int argc, char **argv)
{
  // Set a specific seed to avoid occasional test failures due to
  // statistically unlikely, but possible results.
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
