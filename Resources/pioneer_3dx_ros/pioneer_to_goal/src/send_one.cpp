#include "ros/ros.h"

#include <tf/transform_listener.h>

#include <geometry_msgs/PoseStamped.h>


using namespace std;
string move_base_str;
float poz_x, poz_y, poz_theta;


int main(int argc, char** argv) {

    ros::init(argc, argv, "send_one");
    ros::NodeHandle nh("~");

    ros::Publisher pub_1_ = nh.advertise<geometry_msgs::PoseStamped>( "/pioneer1/move_base_simple/goal", 1 );
    ros::Publisher pub_2_ = nh.advertise<geometry_msgs::PoseStamped>( "/pioneer2/move_base_simple/goal", 1 );

    if (!nh.getParam("poz_x", poz_x))
        poz_x = 0;
    if (!nh.getParam("poz_y", poz_y))
        poz_y = 0;
    if (!nh.getParam("poz_theta", poz_theta))
        poz_theta = 0;
    ROS_INFO("Sending  :x=%f y=%f ",poz_x, poz_y);


    std::string fixed_frame = "map";

    tf::Quaternion quat;

    quat.setRPY(0.0, 0.0, poz_theta);

    tf::Stamped<tf::Pose> p = tf::Stamped<tf::Pose>(tf::Pose(quat, tf::Point(poz_x, poz_y, 0.0)), ros::Time::now(), fixed_frame);

    geometry_msgs::PoseStamped goal;

    tf::poseStampedTFToMsg(p, goal);

    ROS_INFO("Setting goal: Frame:%s, Position(%.3f, %.3f, %.3f), Orientation(%.3f, %.3f, %.3f, %.3f) = Angle: %.3f\n", fixed_frame.c_str(),
        goal.pose.position.x, goal.pose.position.y, goal.pose.position.z,
        goal.pose.orientation.x, goal.pose.orientation.y, goal.pose.orientation.z, goal.pose.orientation.w, poz_theta);
    pub_1_.publish(goal);

    pub_2_.publish(goal);

    ros::spinOnce();

    return 0;
}




