//ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"


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

void visualizeCallback( const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& img)
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
        //cv::hconcat(matScan, cv_ptr->image, matScan);
        //imshow("view", cv_bridge::toCvShare(img, "bgr8")->image);
        imshow("view", cv_ptr->image);


    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
    }

    //visualize
    imshow(window_name, matScan);

    //fill matScan with zeros
    Z = Mat::zeros(matScan.size(), matScan.type());
    Z.copyTo(matScan);

    cv_bridge::CvImagePtr cv_ptr1(new cv_bridge::CvImage);
    ros::Time time = ros::Time::now();
    cv_ptr1->encoding = "bgr8";
    cv_ptr1->header.stamp = time;
    cv_ptr1->header.frame_id = "/laser_out";
    cv_ptr1->image = matScan;

    //image_pub_.publish(cv_ptr->toImageMsg());

    /*
    rows = matScan.rows;
    cols = matScan.cols + cv_ptr->image.cols;
    */
    rows = cv_ptr1->image.rows;
    cols = cv_ptr1->image.cols +cv_ptr->image.cols;

    //ROS_INFO("Laser_rows: %d Laser_cols: %d Image_rows: %d Image_cols %d", matScan.rows, matScan.cols, cv_ptr->image.rows, cv_ptr->image.cols);
    ROS_INFO("max rows: %d max cols: %d ", rows, cols);

    cv_bridge::CvImagePtr result(new cv_bridge::CvImage);
    //result.create(rows, cols, CV_8UC1);
    //matScan.copyTo(result(Rect(0, 0, matScan.cols, matScan.rows)));
    //cv_ptr->image.copyTo(result(Rect(matScan.cols, 0, cv_ptr->image.cols, cv_ptr->image.rows)));

    result->encoding = "bgr8";
    result->header.stamp = time;
    result->header.frame_id = "/all_out";

    cv::hconcat(cv_ptr->image,cv_ptr1->image, result->image);
    imshow("result", result->image);
    /*
    out_msg.header   = img->header; // Same timestamp and tf frame as input image
    out_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1; // Or whatever
    out_msg.image    = result; // Your cv::Mat

    pub.publish(out_msg.toImageMsg());
    */
    isExit();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle nh("~");

    if (!nh.getParam("pioneer_number", pioneer_number))
        pioneer_number = 5;
    window_name = window_name + boost::lexical_cast<std::string>(pioneer_number);
    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow("view", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);

    /*
    ros::Publisher pub;
    pub=nh.advertise<sensor_msgs::Image>("/imagine",1);
    */

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan", 10);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/camera/rgb/image_raw", 10);
    //TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image> sync(laser_sub, image_sub, 10);
    //sync.registerCallback(boost::bind(&visualizeCallback, _1, _2));


    typedef sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(100), laser_sub, image_sub);
    sync.registerCallback(boost::bind(&visualizeCallback, _1, _2));


    printf("wtf bro");
    ros::spin();
    return 0;
}
