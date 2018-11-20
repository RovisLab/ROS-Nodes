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
    ros::NodeHandle nh("~");

    message_filters::Subscriber<sensor_msgs::LaserScan> laser_sub;
    message_filters::Subscriber<sensor_msgs::Image> image_sub;
    TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image> sync;
    int width, height, centerX, centerY, scale;
    Mat matScan;
    cv_bridge::CvImagePtr cv_ptr;
    Mat Z;
    int rows, cols;

    Data_Message_Filters(ros::NodeHandle& _nh):nh(_nh),
                      laser_sub(nh, "/scan" , 10),
                      image_sub(nh, "/pioneer1/camera/rgb/image_raw" , 10),
      sync(laser_sub, image_sub)
    {
            sync.registerCallback(boost::bind(&Data_Message_Filters::visualizeCallback, this, _1, _2));
    }


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
        this->width = 400;
        this->height = 600;
        this->centerX = this->width/2;
        this->centerY = this->height/2;
        this->scale = 40;
        this->matScan.create(this->width, this->height, CV_8UC1);
        //this->matScan = Mat::zeros(this->width, this->height,CV_8UC1);


        for(int i = 0; i < scan->ranges.size(); i++)
        {
            theta = scan->angle_min + (i * scan->angle_increment);
            y = this->scale * scan->ranges[i] * cos(theta);
            x = this->scale * scan->ranges[i] * sin(theta);
            y = this->centerY - y;
            x = this->centerX - x;

            //fill Mat with data
            if(x > 0 && y > 0)
            {
                this->matScan.at<uchar>(y, x) = 255;
            }
        }

        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            resize(this->cv_ptr->image, this->cv_ptr->image, Size(this->matScan.cols, this->matScan.rows));
            imshow("view", this->cv_ptr->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
        }

        //fill matScan with zeros
        this->Z = Mat::zeros(this->matScan.size(), this->matScan.type());
        this->Z.copyTo(this->matScan);
        imshow("result", this->matScan);

        this->isExit();

        this->rows = this->matScan.rows;
        this->cols = this->matScan.cols + this->cv_ptr->image.cols;
    }
    ~Test_Message_Filters()
    {
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

    Data_Message_Filters test_data(nh);
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        ROS_INFO("rows %d and cols %d", listener.rows, listener.cols);
        //ROS_INFO("This is theta1f: %.2f", listener.theta1f);
        //ROS_INFO("This is theta2f: %.2f", listener.theta2f);
        //ROS_INFO("This is theta2f: %.2f", listener.theta2f);
        //if(listener.rows > 0 && listener.cols > 0)
        //    imshow(window_name, listener.matScan);
        //Mat result(listener.rows, listener.cols, CV_8UC1);
        //cv::hconcat(listener.matScan, listener.cv_ptr->image, result);
        //imshow("result", result);
        //waitKey(0);
        imshow("view", listener.cv_ptr->image);
        listener.isExit();



        loop_rate.sleep();
    }

    return 0;
}
