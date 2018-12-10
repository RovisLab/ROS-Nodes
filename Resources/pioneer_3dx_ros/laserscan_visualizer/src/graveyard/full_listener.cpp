//ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "nav_msgs/Odometry.h"

//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//general
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace cv;
using namespace message_filters;

//global
const int width = 400;
const int height = 600;
const int centerX = width/2;
const int centerY = height/2;
int scale = 40;
float theta;
int y, x;
int pioneer_number;
string window_name="LaserScan";
Mat matScan(width, height, CV_8UC1);
cv_bridge::CvImagePtr cv_ptr;
Mat Z, result;
int rows, cols;

void isExit()
{
    //display or exit program if q is pressed
    char key = waitKey(1);
    if(key == 113)
        exit(0);
}

void visualizeCallback(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::LaserScan::ConstPtr& scan, const nav_msgs::Odometry::ConstPtr& odom)
{
    //cv_bridge::CvImage out_msg;

    //data acquisition
    for(int i = 0; i<scan->ranges.size(); i++)
    {
        // angle_min - start angle of the scan
        // angle_increment - change of angle
        theta = scan->angle_min + (i * scan->angle_increment);
        y = scale * scan->ranges[i] * cos(theta);
        x = scale * scan->ranges[i] * sin(theta);
        y = centerY - y;
        x = centerX - x;

        //fill Mat with data
        if(x>0 && y>0)
        {
            matScan.at<uchar>(y, x) = 255;
        }
    }
    try
    {
        cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8); //image encoding: BGR8 is colored.
        resize(cv_ptr->image, cv_ptr->image, Size(matScan.cols, matScan.rows));
        //visualize lasesrscan
        imshow("view", cv_ptr->image);


    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }

    //visualize image
    imshow(window_name, matScan);
    isExit();

    //fill matScan with zeros
    Z = Mat::zeros(matScan.size(), matScan.type());
    Z.copyTo(matScan);

    ROS_INFO("Seq: [%d]", odom->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y);
    ROS_INFO("Orientation->  w: [%f]", odom->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z);



}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "full_visualizer");
    ros::NodeHandle nh("~");

    if (!nh.getParam("pioneer_number", pioneer_number))
        pioneer_number = 5;
    window_name = window_name + boost::lexical_cast<std::string>(pioneer_number);
    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow("view", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_raw", 10);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/odom", 10);
    TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image, nav_msgs::Odometry> sync(laser_sub, image_sub, odom_sub, 10);
    sync.registerCallback(boost::bind(&visualizeCallback, _1, _2, _3));

    ros::spin();
    return 0;
}
