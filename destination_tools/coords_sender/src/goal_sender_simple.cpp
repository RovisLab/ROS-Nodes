#include <ros/ros.h>
#include <coords_msgs/Coords.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_sender_simple");
    ros::NodeHandle nh;
    float a=10, b= 4;

    ros::Publisher pub = nh.advertise<coords_msgs::Coords>("coord_topic", 10,true);
    ros::Rate loop_rate(10);
    while(ros::ok())
    {

        coords_msgs::Coords coordonate;
        coordonate.goal_x = a;
        coordonate.goal_y = b;//or z
        ROS_INFO("se transmit coordonatele %f, %f",coordonate.goal_x,coordonate.goal_y);
        pub.publish(coordonate);
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}



