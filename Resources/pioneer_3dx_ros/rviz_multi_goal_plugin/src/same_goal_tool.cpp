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

#include "same_goal_tool.h"

namespace rviz
{

SameGoalTool::SameGoalTool()
{
  shortcut_key_ = 'b';

  topic_property_1_ = new StringProperty( "Topic1", "goal",
                                        "The topic on which to publish navigation goals.",
                                        getPropertyContainer(), SLOT( updateTopic() ), this );
  topic_property_2_ = new StringProperty( "Topic2", "goal",
                                          "The topic on which to publish navigation goals.",
                                          getPropertyContainer(), SLOT( updateTopic() ), this );
}

void SameGoalTool::onInitialize()
{
  PoseTool::onInitialize();
  setName( "2D Same Goal" );
  updateTopic();
}

void SameGoalTool::updateTopic()
{
  pub_1_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_1_->getStdString(), 1 );
  pub_2_ = nh_.advertise<geometry_msgs::PoseStamped>( topic_property_2_->getStdString(), 1 );
}

void SameGoalTool::onPoseSet(double x, double y, double theta)
{
  std::string fixed_frame = context_->getFixedFrame().toStdString();
  tf::Quaternion quat;
  quat.setRPY(0.0, 0.0, theta);
  tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(x, y, 0.0)), ros::Time::now(), fixed_frame);
  geometry_msgs::PoseStamped goal_1_;
  tf::poseStampedTFToMsg(p, goal_1_);
  goal_1_.pose.position.x = 9;
  goal_1_.pose.position.y = 12;
  goal_1_.pose.orientation.w = 1.57;
  ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
      goal_1_.pose.position.x, goal_1_.pose.position.y, goal_1_.pose.position.z,
      goal_1_.pose.orientation.x, goal_1_.pose.orientation.y, goal_1_.pose.orientation.z, goal_1_.pose.orientation.w, theta);
  pub_1_.publish(goal_1_);
  pub_2_.publish(goal_1_);

}

} // end namespace rviz

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS( rviz::SameGoalTool, rviz::Tool )
