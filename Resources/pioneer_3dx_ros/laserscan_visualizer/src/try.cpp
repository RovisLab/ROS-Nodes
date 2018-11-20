#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"


#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
//OpenCV
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
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

int rows, cols;


class MyClass {
    public:
        MyClass() :
            nh("~"),
            laserscan_sub_( nh, "/scan", 1 ),
            image_sub_( nh, "/camera/rgb/image_raw", 1 ),
            sync( MySyncPolicy(10), laserscan_sub_, image_sub_)
        {
            sync.registerCallback( boost::bind( &MyClass::callback, this, _1, _2 ) );
        }

        void isExit()
        {
            //display or exit program if q is pressed
            char key = waitKey(1);
            if(key == 113)
                exit(0);
        }
        void callback(const sensor_msgs::LaserScanConstPtr& scan, const sensor_msgs::ImageConstPtr& img)
        {
            float theta;
            int y, x;
            Mat matScan(width, height, CV_8UC1);
            cv_bridge::CvImagePtr cv_ptr;
            Mat Z;

            //data acquisition
            for(int i = 0; i < scan->ranges.size(); i++)
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
                //imshow("view", cv_ptr->image);


            }
            catch (cv_bridge::Exception& e)
            {
                ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
            }
            Z = Mat::zeros(matScan.size(), matScan.type());
            Z.copyTo(matScan);
            ROS_INFO("col: %d",matScan.cols);
            //visualize
            //imshow("result", matScan);
            //this->isExit();
        }

    private:
        ros::NodeHandle nh("~");
        typedef message_filters::Subscriber < sensor_msgs::Image > ImageSubscriber;
        typedef message_filters::Subscriber < sensor_msgs::LaserScan > LaserSubscriber;

        ImageSubscriber image_sub_;
        LaserSubscriber laserscan_sub_;

        typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::LaserScan, sensor_msgs::Image> MySyncPolicy;
        message_filters::Synchronizer< MySyncPolicy > sync;
};

int main(int argc, char** argv) {

    ros::init( argc, argv, "my_node" );

    MyClass mc;
    //namedWindow("view", WINDOW_AUTOSIZE);
    //namedWindow("result", WINDOW_AUTOSIZE);

    ros::spin();

    exit(EXIT_SUCCESS);
}
