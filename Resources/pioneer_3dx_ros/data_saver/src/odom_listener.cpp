#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/transform_listener.h>
#include <fstream>

using namespace std;
using namespace nav_msgs;

ofstream myfile_1_1,  myfile_1_2;

string package_string_path= ros::package::getPath("data_saver");

void odom1Cb(const OdometryConstPtr& msg)
{
    tf::Pose pose;
    tf::poseMsgToTF(msg->pose.pose, pose);

    double m_orientation;
    m_orientation = tf::getYaw(pose.getRotation());

    myfile_1_1 << msg->pose.pose.position.x << " " << msg->pose.pose.position.y << "\n";
}
void globalCb(const PathConstPtr& msg)
{
    for (std::vector<geometry_msgs::PoseStamped>::const_iterator it= msg->poses.begin(); it!= msg->poses.end();  ++it)
    {
        myfile_1_2 << it->pose.position.x << " " << it->pose.position.y << "\n";
    }
}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "odom_listener");
        ros::NodeHandle nh("~");

        string ss;

        ss = package_string_path + "/resources/pioneer1_odom.txt";
        myfile_1_1.open(ss.c_str());

        cout << ss<<endl;

        ss = package_string_path + "/resources/pioneer1_path.txt";
        myfile_1_2.open(ss.c_str());

        cout << ss;

        ros::Subscriber path_sub_1 = nh.subscribe("/pioneer1/odom", 1, odom1Cb);
        ros::Subscriber global_sub = nh.subscribe("/pioneer1/move_base/NavfnROS/plan",1,globalCb);
        ros::spin();

        myfile_1_1.close();
        myfile_1_2.close();
        return 0;
}
