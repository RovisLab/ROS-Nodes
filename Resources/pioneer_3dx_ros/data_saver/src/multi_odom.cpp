#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <fstream>

using namespace std;
using namespace nav_msgs;

ofstream myfile_1_1, myfile_1_2;
ofstream myfile_2_1, myfile_2_2;

string package_string_path= ros::package::getPath("data_saver");

void odom1Cb(const OdometryConstPtr& msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    double m_orientation;
    m_orientation = tf::getYaw(pose.getRotation());

    myfile_1_1 << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->twist.twist.linear.x << " " << msg->twist.twist.angular.z << " " << m_orientation <<"\n";
    myfile_1_2 << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";

}

void odom2Cb(const OdometryConstPtr& msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    double m_orientation;
    m_orientation = tf::getYaw(pose.getRotation());

    myfile_2_1 << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << " " << msg->twist.twist.linear.x << " " << msg->twist.twist.angular.z << " " << m_orientation <<"\n";
    myfile_2_2 << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";

}


int main(int argc, char **argv)
{
        ros::init(argc, argv, "multi_odom");
        ros::NodeHandle nh("~");

        string ss;

        ss = package_string_path + "/resources/pioneer1_odom.txt";
        myfile_1_1.open(ss.c_str());

        ss = package_string_path + "/resources/dwa1_path.txt";
        myfile_1_2.open(ss.c_str());

        ss = package_string_path + "/resources/pioneer2_odom.txt";
        myfile_2_1.open(ss.c_str());

        ss = package_string_path + "/resources/dwa2_path.txt";
        myfile_2_2.open(ss.c_str());

        ros::Subscriber path_sub_1 = nh.subscribe("/pioneer1/odom", 1, odom1Cb);
        ros::Subscriber path_sub_2 = nh.subscribe("/pioneer2/odom", 1, odom2Cb);


        ros::spin();

        myfile_1_1.close();
        myfile_1_2.close();
        myfile_2_1.close();
        myfile_2_2.close();

        return 0;
}
