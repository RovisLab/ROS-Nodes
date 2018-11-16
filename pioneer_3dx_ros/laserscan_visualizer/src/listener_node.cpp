//ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "sensor_msgs/Image.h"


//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

//general
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <image_transport/image_transport.h>
#include <cmath>
#include <boost/lexical_cast.hpp>
#include <boost/bind.hpp>

using namespace std;
using namespace cv;
using namespace message_filters;

class Listener
{
public:
    int width, height, centerX, centerY, scale;
    Mat matScan;
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

    void visualizeCallback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& img)
    {
        float theta;
        int y, x;
        width = 400;
        height = 600;
        centerX = width/2;
        centerY = height/2;
        scale = 40;
        matScan.create(this->width, this->height, CV_8UC1);
        //this->matScan = Mat::zeros(this->width, this->height,CV_8UC1);


        for(int i = 0; i < scan->ranges.size(); i++)
        {
            theta = scan->angle_min + (i * scan->angle_increment);
            y = scale * scan->ranges[i] * cos(theta);
            x = scale * scan->ranges[i] * sin(theta);
            y = centerY - y;
            x = centerX - x;

            //fill Mat with data
            if(x > 0 && y > 0)
            {
                matScan.at<uchar>(y, x) = 255;
            }
        }

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            resize(cv_ptr->image, cv_ptr->image, Size(matScan.cols, matScan.rows));
            imshow("view", cv_ptr->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
        }

        //fill matScan with zeros
        Z = Mat::zeros(matScan.size(), matScan.type());
        Z.copyTo(matScan);
        //imshow("result", matScan);

        this->isExit();

        rows = matScan.rows;
        cols = matScan.cols + cv_ptr->image.cols;
        result.create(rows, cols, CV_8UC1);
        cv::hconcat(matScan, cv_ptr->image, result);
        imshow("result", result);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "visualizer");
    ros::NodeHandle nh("~");
    int pioneer_number;
    string window_name="LaserScan";

    if (!nh.getParam("pioneer_number", pioneer_number))
        pioneer_number = 5;
    window_name = window_name + boost::lexical_cast<std::string>(pioneer_number);

    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow("view", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);

    ros::Rate loop_rate(10);

    Listener listener;

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub(nh, "/scan", 100);
    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh, "/pioneer1/camera/rgb/image_raw", 100);
    TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image> sync(laser_sub, image_sub, 200);
    sync.registerCallback(boost::bind(&Listener::visualizeCallback, &listener, _1, _2));

        while (ros::ok())
        {
            ros::spinOnce();
            //ROS_INFO("rows %d and cols %d", listener.rows, listener.cols);
            //ROS_INFO("This is theta1f: %.2f", listener.theta1f);
            //ROS_INFO("This is theta2f: %.2f", listener.theta2f);
            //ROS_INFO("This is theta2f: %.2f", listener.theta2f);
            //if(listener.rows > 0 && listener.cols > 0)
            //    imshow(window_name, listener.matScan);
            //Mat result(listener.rows, listener.cols, CV_8UC1);
            //cv::hconcat(listener.matScan, listener.cv_ptr->image, result);
            //imshow("result", result);
            //waitKey(0);
            //imshow("view", listener.cv_ptr->image);
            //listener.isExit();



            loop_rate.sleep();
        }

    return 0;
}
