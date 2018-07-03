#include "ros/ros.h"
#include <string>
#include <fstream>
#include <geometry_msgs/Twist.h>
#include <stdlib.h>

int main(int argc, char **argv)
{
	if (argc <= 1)
		{
			ROS_INFO_STREAM("****Wrong input***");
			ROS_INFO_STREAM("***Please use a comand like: rosrun trajectory_generation trajectory_publisher <path to the coordinate_file>***");
			return 0;
		}

	ros::init(argc, argv, "publish_velocity");
	ros::NodeHandle n;
	ros::Publisher pub_r1 = n.advertise<geometry_msgs::Twist>("robot1/cmd_vel", 1000);
	ros::Publisher pub_r2 = n.advertise<geometry_msgs::Twist>("robot2/cmd_vel", 1000);
    ros::Rate loop_rate(2);

	
	std::string linie;
	std::fstream myfile;
	myfile.open(argv[1]);
	if (myfile.is_open())
		ROS_INFO_STREAM("Fisierul exista si este deschis ");
	else
		ROS_INFO_STREAM("Fisier inexistent ");
	

	geometry_msgs::Twist msg;
	while (ros::ok()&&getline(myfile,linie))
	{
		msg.linear.x=std::stof(linie.substr(0,linie.find(" ")))/100;
		linie.erase(0,linie.find(" ")+1);
		msg.angular.z=std::stof(linie);		
		
		pub_r1.publish(msg);
		pub_r2.publish(msg);
		ROS_INFO_STREAM("Sending random velocity command:"<<" linear="<<msg.linear.x<<" angular="<<msg.angular.z);
		ros::spinOnce();
		loop_rate.sleep();
	}
	myfile.close();
   return 0;
}
