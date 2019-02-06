/*
 * Copyright (c) 2008, Willow Garage, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Willow Garage, Inc. nor the names of its
 *       contributors may be used to endorse or promote products derived from
 *       this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>

#include "rviz/display_context.h"
#include "rviz/properties/string_property.h"

#include "multi_goal_tool.h"

namespace rviz
{

MultiGoalTool::MultiGoalTool()
{
  shortcut_key_ = 'm';

  topic_property_1_ = new StringProperty( "Topic1", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
  topic_property_2_ = new StringProperty( "Topic2", "goal",
                                          "The topic on which to publish navigation goals.",
                                          getPropertyContainer(), SLOT( updateTopic() ), this );
  topic_property_3_ = new StringProperty( "Topic3", "goal",
                                          "The topic on which to publish navigation goals.",
                                          getPropertyContainer(), SLOT( updateTopic() ), this );
}

void MultiGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "2D Multi-Nav Goal" );
  updateTopic();
}

void MultiGoalTool::updateTopic()
{
  pub_1_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_1_->getStdString(), 1 );
  pub_2_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_2_->getStdString(), 1 );
  pub_3_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_3_->getStdString(), 1 );
}

void MultiGoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal_1_;
  tf::poseStampedTFToMsg(p, goal_1_);
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal_1_.pose.position.x, goal_1_.pose.position.y, goal_1_.pose.position.z,
      goal_1_.pose.orientation.x, goal_1_.pose.orientation.y, goal_1_.pose.orientation.z, goal_1_.pose.orientation.w, theta);
  pub_1_.publish(goal_1_);

  geometry_msgs::PoseStamped goal_2_;
  tf::poseStampedTFToMsg(p, goal_2_);
  goal_2_.pose.position.y -= 1;
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal_2_.pose.position.x, goal_2_.pose.position.y, goal_2_.pose.position.z,
      goal_2_.pose.orientation.x, goal_2_.pose.orientation.y, goal_2_.pose.orientation.z, goal_2_.pose.orientation.w, theta);
  pub_2_.publish(goal_2_);

  geometry_msgs::PoseStamped goal_3_;
  tf::poseStampedTFToMsg(p, goal_3_);
  goal_3_.pose.position.y -= 2;
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal_3_.pose.position.x, goal_3_.pose.position.y, goal_3_.pose.position.z,
      goal_3_.pose.orientation.x, goal_3_.pose.orientation.y, goal_3_.pose.orientation.z, goal_3_.pose.orientation.w, theta);
  pub_3_.publish(goal_3_);
}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::MultiGoalTool, rviz::Tool )
