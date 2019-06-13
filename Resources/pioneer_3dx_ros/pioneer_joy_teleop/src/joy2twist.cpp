#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

class Joy2twist
{
private:
  ros::NodeHandle nh_;
  ros::Publisher  pubTwist_;
  ros::Subscriber subJoy_;


public:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& msg);
  Joy2twist();
};

Joy2twist::Joy2twist()
{
    subJoy_   = nh_.subscribe("joy", 1000, &Joy2twist::joyCallback, this);
    pubTwist_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1000);
}

void Joy2twist::joyCallback(const sensor_msgs::Joy::ConstPtr& msg)
{
  geometry_msgs::Twist out_cmd_vel;

  out_cmd_vel.angular.z = msg->axes[3];
  out_cmd_vel.linear.x = msg->axes[4];

  pubTwist_.publish(out_cmd_vel);
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "joy2twist_node");

  Joy2twist joy2twist;

  ros::spin();
}

