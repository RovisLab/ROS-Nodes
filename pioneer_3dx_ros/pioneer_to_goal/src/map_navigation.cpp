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


//#define PI 3.14159265358979

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;


string move_base_str;
float poz_x, poz_y, poz_theta;


int main(int argc, char** argv) {

        ros::init(argc, argv, "map_navigation");
        ros::NodeHandle nh("~");

        if (!nh.getParam("poz_x", poz_x))
            poz_x = 0;
        if (!nh.getParam("poz_y", poz_y))
            poz_y = 0;
        if (!nh.getParam("poz_theta", poz_theta))
            poz_theta = 0;
         ROS_INFO("Sending  :x=%f y=%f ",poz_x, poz_y);
/*
        move_base_str = "/move_base_simple/goal";
*/
        actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base",true) ;

        while(!ac.waitForServer(ros::Duration(10.0))){
            ROS_INFO("Waiting for the move_base action server to come up");
        }
        ROS_INFO("Connected to move base server");

        // Send a goal to move_base
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();

        goal.target_pose.pose.position.x = poz_x;
        goal.target_pose.pose.position.y = poz_y;
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

        ros::spinOnce();

	return 0;
}



