#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <math.h>

using namespace std;
using namespace nav_msgs;

string package_string_path= ros::package::getPath("data_saver");

 Path pathRead(ifstream& fisier)
{
     float a = 0;
     float b = 0;

     Path plan;
     plan.header.frame_id = "/map";
     plan.header.stamp = ros::Time::now();

     geometry_msgs::PoseStamped point;
     point.header.frame_id = "/map";
     point.header.stamp = ros::Time::now();

     while(fisier >> a && fisier >> b)
     {
         point.pose.position.x = a;
         point.pose.position.y = b;
         plan.poses.push_back(point);
     }

     float path_length = 0.0;

     for( int i = 0; i < (plan.poses.size() -1); ++i)
     {
         path_length += hypot(  (plan.poses[i+1].pose.position.x - plan.poses[i].pose.position.x),
                                  (plan.poses[i+1].pose.position.y - plan.poses[i].pose.position.y ));
     }

     ROS_INFO("... length: %6.3f", path_length);
     return plan;

}

int main(int argc, char **argv)
{
        ros::init(argc, argv, "odom_path");
        ros::NodeHandle nh("~");
        ros::Rate loop_rate(10);
        ifstream plan_path_1, dwa_path_1;

        string ss;

        ss = package_string_path + "/resources/pioneer1_path.txt";
        plan_path_1.open(ss.c_str());

        ss = package_string_path + "/resources/pioneer1_odom.txt";
        dwa_path_1.open(ss.c_str());

        ros::Publisher path_pub_1_ = nh.advertise <Path>("/pioneer_path_1", 10);
        ros::Publisher dwa_pub_1_ = nh.advertise <Path>("/dwa_path_1", 10);

        Path path_plan_1_, dwa_plan_1_;
        path_plan_1_ = pathRead(plan_path_1);
        dwa_plan_1_ = pathRead(dwa_path_1);

        while (ros::ok())
          {

            path_pub_1_.publish(path_plan_1_);
            dwa_pub_1_.publish(dwa_plan_1_);

            ros::spinOnce();

            loop_rate.sleep();
          }

        ros::spin();

        plan_path_1.close();
        dwa_path_1.close();

        return 0;
}
