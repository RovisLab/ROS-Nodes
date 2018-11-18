/*
 * send_goals.cpp
 *
 *  Created on: Apr 7, 2014
 *      Author: roiyeho
 */

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/transform_datatypes.h>
#include <darknet_ros_msgs/BoundingBoxes.h>


//#define PI 3.14159265358979

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;


string move_base_str;
float poz_x, poz_y, poz_theta;
float x=0, z=0;
string st1="person";

void CoordsCallback(const darknet_ros_msgs::BoundingBoxes& msg)
{
    if (st1.compare(msg.bounding_boxes[0].Class) == 0)

        if((msg.bounding_boxes[0].X < 0.8 * x) || (msg.bounding_boxes[0].X > 1.2 * x))
            if((msg.bounding_boxes[0].Z < 0.8 * z) || (msg.bounding_boxes[0].Z > 1.2 * z ))
            {
                actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true) ;

                while(!ac.waitForServer(ros::Duration(10.0)))
                {
                    ROS_INFO("Waiting for the move_base action server to come up");
                }
                ROS_INFO("Connected to move base server");

                // Send a goal to move_base
                move_base_msgs::MoveBaseGoal goal;
                goal.target_pose.header.frame_id = "map";
                goal.target_pose.header.stamp = ros::Time::now();

                goal.target_pose.pose.position.x = msg.bounding_boxes[0].X + poz_x;
                goal.target_pose.pose.position.y = msg.bounding_boxes[0].Z + poz_y;
                goal.target_pose.pose.position.z = 0;

                // Convert the Euler angle to quaternion
                double radians = poz_theta * (M_PI/180);
                tf::Quaternion quaternion;
                quaternion = tf::createQuaternionFromYaw(radians);

                geometry_msgs::Quaternion qMsg;
                tf::quaternionTFToMsg(quaternion, qMsg);
                goal.target_pose.pose.orientation = qMsg;

                ROS_INFO("Sending goal :x=%f y=%f ", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y);
                ac.sendGoal(goal);

                // Wait for the action to return
                ac.waitForResult();

                if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
                    ROS_INFO("Robot has reached the goal!");
                else
                    ROS_INFO("The base of robot failed for some reason");
                x = msg.bounding_boxes[0].X;
                z = msg.bounding_boxes[0].Z;
            }
}

int main(int argc, char** argv) {

        ros::init(argc, argv, "map_navigation");
        ros::NodeHandle nh("~");

        if (!nh.getParam("poz_x", poz_x))
            poz_x = 0;
        if (!nh.getParam("poz_y", poz_y))
            poz_y = 0;
        if (!nh.getParam("poz_theta", poz_theta))
            poz_theta = 0;

        ros::Subscriber sub = nh.subscribe("/darknet_ros/bounding_boxes",10,CoordsCallback);

        ros::spin();

	return 0;
}



