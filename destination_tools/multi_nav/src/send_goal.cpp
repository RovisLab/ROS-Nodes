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
#include <coords_msgs/Coords.h>

#define PI 3.14159265358979

//typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

    // boss,rosspin!!!

string move_base_str;
float poz_x, poz_y, poz_theta;

void poseCallback(const coords_msgs::Coords& msg)
{
    //tell the action client that we want to spin a thread by default
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac(move_base_str,true) ;

    while(!ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Connected to move base server");

    // Send a goal to move_base
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = msg.goal_x + poz_x;
    goal.target_pose.pose.position.y = msg.goal_y + poz_y;
    goal.target_pose.pose.position.z = 0;

    // Convert the Euler angle to quaternion
    double radians = poz_theta * (M_PI/180);
    tf::Quaternion quaternion;
    quaternion = tf::createQuaternionFromYaw(radians);

    geometry_msgs::Quaternion qMsg;
    tf::quaternionTFToMsg(quaternion, qMsg);
    goal.target_pose.pose.orientation = qMsg;

    ROS_INFO("Sending goal to robot ");
    ac.sendGoal(goal);

    // Wait for the action to return
    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
        ROS_INFO("Robot has reached the goal!");
    else
        ROS_INFO("The base of robot failed for some reason");

}


int main(int argc, char** argv) {

	if (argc < 2) {
		ROS_ERROR("You must specify leader robot id.");
		return -1;
	}

	char *robot_id = argv[1];

	ros::init(argc, argv, "send_goals");
	ros::NodeHandle nh;

        if (!nh.getParam("poz_x", poz_x))
            poz_x = 0;
        if (!nh.getParam("poz_y", poz_y))
            poz_y = 0;
        if (!nh.getParam("poz_theta", poz_theta))
            poz_theta = 0;

        // Create the string "pioneer_X/move_base_simple/goal"
        move_base_str = "/pioneer";
        move_base_str += robot_id;
        move_base_str += "/move_base_simple/goal";
        ROS_INFO("robot topic string is: %s",move_base_str.c_str());

        ros::Subscriber sub = nh.subscribe("coord_topic",10,poseCallback);
        ros::spin();

	return 0;
}



