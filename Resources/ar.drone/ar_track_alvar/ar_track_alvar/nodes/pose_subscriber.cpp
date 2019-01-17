#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/tf.h>  
#include <tf/transform_datatypes.h>

void printPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{   
	//ROS_INFO("I can hear");

    tf::Pose marker_pose_in_camera_;   
	for (int i=0; i < msg->markers.size(); i++)
    {
	marker_pose_in_camera_.setOrigin(tf::Vector3(msg->markers[i].pose.pose.position.x,
                             msg->markers[i].pose.pose.position.y,
                             msg->markers[i].pose.pose.position.z));

	ROS_INFO("x: [%lf]", msg->markers[i].pose.pose.position.x);
	ROS_INFO("y: [%lf]", msg->markers[i].pose.pose.position.y);
	ROS_INFO("z: [%lf]", msg->markers[i].pose.pose.position.z);
    }

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "pose_subscriber");
    ros::NodeHandle nh;
    ros::Subscriber pose_sub = nh.subscribe("ar_pose_marker", 100, printPose);
    ros::spin();

    return 0;

}
