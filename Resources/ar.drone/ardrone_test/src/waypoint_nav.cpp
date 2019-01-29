#include <ros/ros.h>
#include <geometry_msgs/Twist.h>            //for command velocity
#include <geometry_msgs/Vector3.h>          //for command velocity
#include "std_msgs/Empty.h"                 //For take off and landing
#include <ardrone_autonomy/CamSelect.h>     // For toggling camera
#include <ardrone_autonomy/Navdata.h>       //Accessing ardrone's published data
#include <ardrone_autonomy/vector31.h>
#include <ardrone_autonomy/vector21.h>
#include "sensor_msgs/Imu.h"
#include "nav_msgs/Odometry.h"
#include <sensor_msgs/Joy.h>
#include <stdio.h>
#include <math.h>
#include <termios.h>
#include <sstream>
#include <iostream>
#include <fstream>
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "ardrone_test/Drone_odo.h"
#include "ardrone_test/est_co.h"

//subscriber
#include <ros/ros.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>

std_msgs::Empty emp_msg;			// variable in the take off and landing class
geometry_msgs::Twist vel_msg;			// variable in the command velocity class

ros::Publisher T_pub_empty;			//take off publisher
ros::Publisher L_pub_empty;			//landing publisher
ros::Publisher E_pub_empty;
ros::Publisher velocity_publisher;		// velocity publisher
ros::Subscriber pose_subscriber;		// ardrone navdata subsciber
ros::Subscriber imu_subscriber;                 // ardrone imu data subsciber
ros::ServiceClient client1;                     // Variables for Service  // ardrone camera service

using namespace std;

const double PI = 3.14159265359;
double px=0;
double py=0;
double pz=0;
double dist;
float lx;
float ly;
float lz;
float ax;
float ay;
float az;
int f = 200;
double Kp = 0.6, Ki = 0.05; //0.5, 0.01

//class instances
ardrone_autonomy::Navdata drone_navdata;  	// Using this class variable for storing navdata
sensor_msgs::Imu ang;
nav_msgs::Odometry odo;

//callback function declarations
double getDistance(double x1, double y1, double z1, double x2, double y2, double z2);
void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message);		// drone actual data
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message);                       // drone actual data
int getch();
void reset();
void hover(int timee);
void takeoff();
void land();
void move(float lx, float ly, float lz, float ax, float ay, float az );                 // publishing command velocity
void basic_movement();
void printPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg);
void pid(double setpoint_x, double setpoint_y);
void sus();
void jos();


int main(int argc, char **argv)
{
    //Initiate the ROS
    ros::init(argc, argv, "waypoint_nav");
    ros::NodeHandle n;                                                          // Nodehandle is like a class where you are operating

    // Publishing the data
    velocity_publisher = n.advertise<geometry_msgs::Twist>("/cmd_vel", 100);    // initialize to send the Control Input
    T_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/takeoff", 1);          //initialize to send the take off command  /* Message queue length is just 1 */
    L_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/land", 1);             //initialize to send the land command /* Message queue length is just 1 */
    E_pub_empty = n.advertise<std_msgs::Empty>("/ardrone/reset", 1);

    // Subscribing the data
    ros::Subscriber pose_sub = n.subscribe("ar_pose_marker", 100, printPose);
    pose_subscriber = n.subscribe("/ardrone/navdata", 200, poseCallback);	//initialize to receive processed sensor data
    imu_subscriber = n.subscribe("/ardrone/imu", 200, imuCallback);             //initialize to receive raw sensor data



    while (ros::ok())
    {
        ros::Rate loop_rate(10);
        /*cout << "Press key" << endl;
        cout << "w-takeoff | s-land | m-basic movements" << endl;
        cout << "h-hover for input time | x - exit | e - emergency" << endl;*/
        //cout << "m - start drona, s - PID, x - stop drona" << endl;
        cout << "w - start drona, s - stop drona, r - reset, p - PID, h - hover , m - menu" << endl;

        //ros::Subscriber pose_sub = n.subscribe("ar_pose_marker", 100, printPose);

        int c = getch();    // call your non-blocking input function
        cout << endl;
        //int timee;
        int d = 0;

        switch (c)
        {
            case 'm':   cout << "\n Basic movement menu" << endl;
                        basic_movement();
                        break;                                                  //adaugat

            case 's':   //cout << "\n Landing initiated" << endl;
                        land();
                        break;

            case 'w':   cout << "\n Take off initiated" << endl;
                        takeoff();
                        hover(2);
                        break;

            case 'p':   cout << "\n PID" << endl;
                        pid(0.0,0.0);
                        //pid(-0.03, -0.02);
                        break;

            case 'r':   cout << "\n Reset.." << endl;
                        reset();
                        break;

            case 'h':   cout << "\n Hover" << endl;
                        hover(5);
                        break;

            default:    land();
                        break;

        }

        ros::spinOnce();    //if this function is not mentioned the function will stay in the buffer and will not be able to publish
        loop_rate.sleep();

    }

    //land();
    cout<<"landed";

    return 0;

}




void pid(double setpoint_x, double setpoint_y)
{
    double integral_old_x = 0, integral_old_y = 0;
    double val_x, val_y;
    double error_x, error_y;
    double prop_x, prop_y;
    double integral_x, integral_y;
    double min = -1, max = 1;

    ros::Rate loop_rate(5);
    while (ros::ok())
    {
        double init_time=ros::Time::now().toSec();
        double time;
        while (time < (init_time+3.0))                 // Send command for 2 seconds
        {
            //pt x
            error_x = setpoint_x - px;
            prop_x = Kp * error_x;
            integral_x = integral_old_x + Ki * error_x;
            val_x = prop_x + integral_x;

            //pt y
            error_y = setpoint_y - py;
            prop_y = Kp * error_y;
            integral_y = integral_old_y + Ki * error_y;
            val_y = prop_y + integral_y;

            if(val_x < min)
                val_x = min;
            else if(val_x > max)
                val_x = max;

            if(val_y < min)
                val_y = min;
            else if(val_y > max)
                val_y = max;

            //move(val_x, val_y, 0, 0, 0, 0);

            ROS_INFO("x = %lf", px);
            ROS_INFO("val_x: %lf", val_x);
            //ROS_INFO("y = %lf", py);
            //ROS_INFO("val_y: %lf", val_y);
            ros::spinOnce();
            loop_rate.sleep();

            time = ros::Time::now().toSec();
            integral_old_x = integral_x;
            integral_old_y = integral_y;


        }
        break;
    }

}



void reset()
{
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        double init_time=ros::Time::now().toSec();
        double time;
        while (time < (init_time+2.0))                 // Send command for five seconds
        {
            E_pub_empty.publish(emp_msg);              // lands the drone
            ros::spinOnce();
            loop_rate.sleep();
            time = ros::Time::now().toSec();
        }
        ROS_INFO("ARdrone reset");
        break;
    }
}


//subscriber callback
void printPose(const ar_track_alvar_msgs::AlvarMarkers::ConstPtr& msg)
{
    tf::Pose marker_pose_in_camera_;

    for (int i=0; i < msg->markers.size(); i++)
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


void basic_movement()
{
    ros::Rate loop_rate(10);
    int exit_val=0;
    //cout << "Press any key" << endl;
    //cout << "q |up|       i        |forward|             e |exit to prev menu|" << endl;
    //cout << "a |down|    jkl  |left|  back |right|       s |land| other |hover|" << endl;

    cout << endl;
    cout << "p- pid, e- exit menu, s- land" << endl;
    cout << "q- up, a- down, i- forward, k- backward, j- left, l- right" << endl;
	
    while (exit_val==0)
    {
        int l = getch();                            //calling non blocking input fucntion
        cout << endl;
        int sval = 0.5;

        if (l=='q')
        {
            move(0,0,1,0,0,0);
        }
        else if (l=='a')
        {
            move(0,0,-1,0,0,0);
        }
        else if (l=='i')
        {
            move(0.5,0,0,0,0,0);
        }
        else if (l=='k')
        {
            move(-0.5,0,0,0,0,0);
        }
        else if (l=='j')
        {
            move(0,0.5,0,0,0,0);
        }
        else if (l=='l')
        {
            move(0,-0.5,0,0,0,0);
        }
        else if (l=='e')
        {
            exit_val = 1;
        }
        else if (l=='s')
        {
            land();
        }
        else if (l=='p')
        {
            pid(-0.03, -0.02);
            //pid(0.0, 0.0);
        }
        else
        {
            hover(2);
        }

        ros::spinOnce();            //if this function is not mentioned the function will stay in the buffer and cannot be able to publish
        loop_rate.sleep();
    }
}

void poseCallback(const ardrone_autonomy::Navdata::ConstPtr & pose_message)
{
        drone_navdata.vx             = 	pose_message->vx;
        drone_navdata.vy             = 	pose_message->vy;
        drone_navdata.vz             = 	pose_message->vz;
        drone_navdata.ax             = 	pose_message->ax;
        drone_navdata.ay             = 	pose_message->ay;
        drone_navdata.az             = 	pose_message->az;
        drone_navdata.rotX           = 	pose_message->rotX;
        drone_navdata.rotY           = 	pose_message->rotY;
        drone_navdata.rotZ           = 	pose_message->rotZ;
        drone_navdata.magX           = 	pose_message->magX;
        drone_navdata.magY           = 	pose_message->magY;
        drone_navdata.magZ           = 	pose_message->magZ;
        drone_navdata.altd           = 	pose_message->altd;
        drone_navdata.tm             = 	pose_message->tm;
        drone_navdata.header         = 	pose_message->header;
        drone_navdata.batteryPercent =  pose_message->batteryPercent;

}
void imuCallback(const sensor_msgs::Imu::ConstPtr & imu_message)
{
        ang.header              = imu_message->header;
        ang.angular_velocity	= imu_message->angular_velocity;
        ang.linear_acceleration = imu_message->linear_acceleration;
}

int getch()
{
  static struct termios oldt, newt;
  tcgetattr( STDIN_FILENO, &oldt);           // save old settings
  newt = oldt;
  newt.c_lflag &= ~(ICANON);                 // disable buffering
  tcsetattr( STDIN_FILENO, TCSANOW, &newt);  // apply new settings
  int c = getchar();                         // read character (non-blocking)
  tcsetattr( STDIN_FILENO, TCSANOW, &oldt);  // restore old settings

  return c;
}

double deg2rad(double angle_in_degrees)
{
    return angle_in_degrees*PI/180.0;
}

double getDistance(double x1, double y1, double z1, double x2, double y2, double z2)
{
    return sqrt(pow(x1-x2, 2) + pow(y1-y2, 2) + pow(z1-z2, 2));
}

void takeoff()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        double init_time=ros::Time::now().toSec();  // epoch time
        double time;

        while (time < (init_time + 3.0))            // Send command for five seconds
        {
            T_pub_empty.publish(emp_msg);           // launches the drone
            ros::spinOnce();                        // feedback
            loop_rate.sleep();
            time = ros::Time::now().toSec();
        }

        ROS_INFO("ARdrone launched");
        break;
     }
}

void land()
{
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        double init_time=ros::Time::now().toSec();
        double time;

        while (time < (init_time+2.0))                 // Send command for five seconds
        {
            L_pub_empty.publish(emp_msg);              // lands the drone
            ros::spinOnce();
            loop_rate.sleep();
            time = ros::Time::now().toSec();
        }

        ROS_INFO("ARdrone landed");
        break;
    }
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

    //ROS_INFO("vel_mes = %lf", vel_msg.linear.y);

    //defining the angular velocity
    vel_msg.angular.x = ax;
    vel_msg.angular.y = ay;
    vel_msg.angular.z = az;

    velocity_publisher.publish(vel_msg);
}
