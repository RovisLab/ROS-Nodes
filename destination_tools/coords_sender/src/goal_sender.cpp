#include <ros/ros.h>
#include <coords_msgs/Coords.h>

int main(int argc, char **argv) {

    ros::init(argc, argv, "goal_sender");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<coords_msgs::Coords>("coord_topic", 10,true);
    ros::Rate loop_rate(10);

    float a=10.2,b=-1.3;
    while (ros::ok())
    {
        coords_msgs::Coords msg;
        msg.goal_x = a;
        msg.goal_y = b;
        ROS_INFO("se transmit coordonatele %f, %f",msg.goal_x,msg.goal_y);
        pub.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}



