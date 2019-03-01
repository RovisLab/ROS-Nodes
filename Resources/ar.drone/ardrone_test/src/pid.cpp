#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Vector3.h>
#include <sstream>
#include <iostream>
#include <stdio.h>
#include <termios.h>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"

//subscriber
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

using namespace std;

ros::Publisher velocity_publisher;
geometry_msgs::Twist vel_msg;

void pid_control(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
void hover(int timee);
void move(float lx, float ly, float lz, float ax, float ay, float az );


double px=0;
double py=0;
double pz=0;
double stop=0;
double Kp = 0.3, Ki = -0.04;



int main(int argc, char **argv)
{
    //Initiate the ROS
    ros::init(argc, argv, "pid");
    ros::NodeHandle n;

    velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    ros::Subscriber pose_sub = n.subscribe("ar_pose_marker", 100, pid_control);

    ros::spin();

    return 0;

}



/*void pid(double setpoint_x, double setpoint_y)
{
    //double integral_old_x = 0, integral_old_y = 0;
    double val_x=0, val_y=0, val_z=0;
    double error_x=0, error_y=0;
    double prop_x=0, prop_y=0;
    double integral_x=0, integral_y=0;
    double min = -0.3, max = 0.3;

    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        double init_time=ros::Time::now().toSec();
        double time;

        while (time < (init_time+1.0))                 // Send command for 2 seconds, 1.0 -> 1 sec
        {
            time=ros::Time::now().toSec();

            //pt x
            error_x = setpoint_x - px;
            prop_x = Kp * error_x;
            integral_x += Ki * error_x;
            val_x = prop_x + integral_x;

            //pt y
            error_y = setpoint_y - py;
            prop_y = Kp * error_y;
            integral_y += Ki * error_y;
            val_y = prop_y + integral_y;

            if(val_x < min)
                val_x = min;
            else if(val_x > max)
                val_x = max;

            if(val_y < min)
                val_y = min;
            else if(val_y > max)
                val_y = max;

            if(pz <= -1.7)
                val_z = 0.1;
            else if(pz > -0.6)
                val_z = -0.1;
            else
                val_z = 0;

            //if(val_x<0.02 || val_x>-0.1 || val_y<0.04 || val_y>-0.04)
            //if(px<0.02 || px>-0.06 || py<0.04 || py>-0.04)
            if(stop==1)
            {
                hover(1);       //stop drone if marker is not found
                px = 0;
                val_x = 0;
                py = 0;
                val_y = 0;
            }                    //0.03           +-0.04
            else if(px>-0.06 && px<0.0 || py<0.01 && py>-0.01)
            {
                hover(1);
                cout << "hoverig" << endl;
            }
            //else
               // move(val_x, val_y, val_z, 0, 0, 0);


            ROS_INFO("x = %lf", px);
            ROS_INFO("val_x: %lf", val_x);
            //ROS_INFO("y = %lf", py);
            //ROS_INFO("val_y: %lf", val_y);
            //ROS_INFO("z: %lf", pz);
            //ROS_INFO("val_z: %lf", val_z);

            ros::spinOnce();
            loop_rate.sleep();

            time = ros::Time::now().toSec();

        }
        break;
    }

}*/

void pid_control(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    tf::Pose marker_pose_in_camera_;

    if(msg->markers.empty())
    {
        stop = 1;
        ROS_INFO("stop");
    }
    else
    {
        stop = 0;
        for (int i = 0; i < msg->markers.size(); i++)
        {
            marker_pose_in_camera_.setOrigin(tf::Vector3(msg->markers[i].pose.pose.position.x,
                             msg->markers[i].pose.pose.position.y,
                             msg->markers[i].pose.pose.position.z));

            //ROS_INFO("x: [%lf]", msg->markers[i].pose.pose.position.x);
            //ROS_INFO("y: [%lf]", msg->markers[i].pose.pose.position.y);
            //ROS_INFO("z: [%lf]", msg->markers[i].pose.pose.position.z);

            px = msg->markers[i].pose.pose.position.x;
            py = msg->markers[i].pose.pose.position.y;
            pz = msg->markers[i].pose.pose.position.z;
        }

    }
                        //0.02
    double setpoint_x = 0.0, setpoint_y = 0.0;
    double val_x=0, val_y=0, val_z=0;
    double error_x=0, error_y=0;
    double prop_x=0, prop_y=0;
    double integral_x=0, integral_y=0;
    double min = -0.3, max = 0.3;

        //pt x
        error_x = setpoint_x - px;
        prop_x = Kp * error_x;
        integral_x += Ki * error_x;
        val_x = prop_x + integral_x;

        //pt y
        error_y = setpoint_y - py;
        prop_y = Kp * error_y;
        integral_y += Ki * error_y;
        val_y = prop_y + integral_y;

        if(val_x < min)
            val_x = min;
        else if(val_x > max)
            val_x = max;

        if(val_y < min)
            val_y = min;
        else if(val_y > max)
            val_y = max;

        if(pz <= -1.9)
            val_z = -0.1;   //0.1
        else if(pz > -1.4)
            val_z = 0.2;    //-0.15
        else
            val_z = 0;

        //if(val_x<0.02 || val_x>-0.1 || val_y<0.04 || val_y>-0.04)
        //if(px<0.02 || px>-0.06 || py<0.04 || py>-0.04)
        if(stop==1)
        {
            //move(0, 0,val_z, 0, 0, 0);
            hover(1);       //stop drone if marker is not found
            px = 0;
            val_x = 0;
            py = 0;
            val_y = 0;
            pz = 0;
            val_z = 0;
        }                    //0.03           +-0.04
        else if(px>-0.02 && px<0.0 || py<0.01 && py>-0.01)
        //else if(px>-0.03 && px<0.0)
        {
            //move(0, 0, val_z, 0, 0, 0);
            hover(1);
            //ROS_INFO("hovering");
        }
        else
            move(val_x, val_y, val_z, 0, 0, 0);


        ROS_INFO("x = %lf", px);
        ROS_INFO("val_x: %lf", val_x);
        ROS_INFO("y = %lf", py);
        ROS_INFO("val_y: %lf", val_y);
        ROS_INFO("z: %lf", pz);
        ROS_INFO("val_z: %lf", val_z);

}


void hover(int timee)
{
    double t0 = ros::Time::now().toSec();               //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'
    double t1;
    ros::Rate loop_rate(200);
    do
    {
        t1 = ros::Time::now().toSec();                   //ros has a 'Time' function in which the current time can be found by using 'now'.                                         							we need time in sec to compute hence 'tosec'

        vel_msg.linear.x = 0;
        vel_msg.linear.y = 0;
        vel_msg.linear.z = 0;

        vel_msg.angular.x = 0;
        vel_msg.angular.y = 0;
        vel_msg.angular.z = 0;

        velocity_publisher.publish(vel_msg);

        ros::spinOnce();                                 //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
        loop_rate.sleep();
    }
    while(t1 <= (t0+timee));
}


void move(float lx, float ly, float lz, float ax, float ay, float az )
{
    //defining the linear velocity
    vel_msg.linear.x = lx;
    vel_msg.linear.y = ly;
    vel_msg.linear.z = lz;

    //defining the angular velocity
    vel_msg.angular.x = ax;
    vel_msg.angular.y = ay;
    vel_msg.angular.z = az;

    velocity_publisher.publish(vel_msg);
}
