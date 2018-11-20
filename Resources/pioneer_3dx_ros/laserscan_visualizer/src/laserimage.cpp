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
    ros::NodeHandle nh;

    message_filters::Subscriber<sensor_msgs::LaserScan> sub_laser;
    message_filters::Subscriber<sensor_msgs::Image> sub_image;
    message_filters::TimeSynchronizer<sensor_msgs::LaserScan, sensor_msgs::Image> sync;


    Listener(ros::NodeHandle& _nh): nh(_nh),
            sub_laser(_nh, "/scan" , 10),
            sub_image(_nh, "pioneer1/camera/rgb/image_raw" , 10),
            sync(_nh, &Listener::sub_laser, &Listener::sub_image)
    {
            sub_laser.registerCallback(&Listener::laserScanCallback, this);
            sub_image.registerCallback(&Listener::imageCallback, this);
            sync.registerCallback(boost::bind(&Listener::sync_subs_callback, this, _1, _2));
    }



    void sync_subs_callback(const sensor_msgs::LaserScan::ConstPtr& scan, const sensor_msgs::ImageConstPtr& img)
    {
        rows = matScan.rows;
        cols = matScan.cols + cv_ptr->image.cols;
        result.create(rows, cols, CV_8UC1);
        cv::hconcat(matScan, cv_ptr->image, result);
        imshow("test", result);
        isExit();
    }

    void isExit()
    {
        //display or exit program if q is pressed
        char key = waitKey(1);
        if(key == 113)
            exit(0);
    }

    void imageCallback(const sensor_msgs::ImageConstPtr& img)
    {
        try
        {
            cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            //resize(cv_ptr->image, cv_ptr->image, Size(matScan.cols, matScan.rows));
            imshow("view", cv_ptr->image);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
        }
        this->isExit();
    }

    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan)
    {
        float theta;
        int y, x;
        width = 400;
        height = 600;
        centerX = width/2;
        centerY = height/2;
        scale = 40;
        matScan.create(this->width, this->height, CV_8UC1);
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
        //fill matScan with zeros
        Z = Mat::zeros(matScan.size(), matScan.type());
        Z.copyTo(matScan);
        imshow("result", matScan);

        this->isExit();

    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "lasimg");
    ros::NodeHandle nh;
    int pioneer_number;
    string window_name="LaserScan";

    if (!nh.getParam("pioneer_number", pioneer_number))
        pioneer_number = 5;
    window_name = window_name + boost::lexical_cast<std::string>(pioneer_number);

    namedWindow(window_name, WINDOW_AUTOSIZE);
    namedWindow("view", WINDOW_AUTOSIZE);
    namedWindow("result", WINDOW_AUTOSIZE);
    namedWindow("test", WINDOW_AUTOSIZE);

    Listener test_message_filters(nh);

    ros::Rate loop_rate(10);
    while(ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
