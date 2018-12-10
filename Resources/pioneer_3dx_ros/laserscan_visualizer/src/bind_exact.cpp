//ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <ros/package.h>
#include <nav_msgs/Odometry.h>

#include <fstream>
#include <locale>
#include <string>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>
#include <sensor_msgs/Image.h>

using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;


//global
const int width = 240;
const int height = 320;
const int centerX = width/2;
const int centerY = height/2;
int scale = 40;
float theta;
int y, x;
Mat matScan(width, height, CV_8UC1);
Mat Z;
int pioneer_number;
string window_name="LaserScan";

string path = ros::package::getPath("laserscan_visualizer");

string filename_image, filename_laser;
string file_path = path + "/state_run2.txt";

ofstream myfile;

void bindCallback( const ImageConstPtr& image, const LaserScanConstPtr& scan, const OdometryConstPtr& odom )
{
    myfile << ". " << odom->pose.pose.position.x << "\t" << odom->pose.pose.position.y << "\t" << odom->pose.pose.orientation.w << "\t" << odom->twist.twist.linear.x << "\t" << odom->twist.twist.angular.z << "\t";

    /*
For popular image encodings, CvBridge will optionally do color or pixel depth conversions as necessary. To use this feature, specify the encoding to be one of the following strings:

    mono8: CV_8UC1, grayscale image

    mono16: CV_16UC1, 16-bit grayscale image

    bgr8: CV_8UC3, color image with blue-green-red color order

    rgb8: CV_8UC3, color image with red-green-blue color order

    bgra8: CV_8UC4, BGR color image with an alpha channel

    rgba8: CV_8UC4, RGB color image with an alpha channel

Note that mono8 and bgr8 are the two image encodings expected by most OpenCV functions.
    */
    cv_bridge::CvImagePtr cv_ptr;

    long time = image->header.stamp.sec * 1000 + image->header.stamp.nsec;

    std::string time_string = boost::lexical_cast<std::string>(time);

    filename_image = path + "/image_raw_laser/image_raw_" + time_string + ".png";

    string file_image = "image_raw_" + time_string + ".png";
    myfile << file_image << "\t";

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5);

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        resize(cv_ptr->image, cv_ptr->image, Size(height, width));
        imwrite(filename_image, cv_ptr->image, compression_params);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image->encoding.c_str());
    }

    //data acquisition
    for(int j = 0; j < scan->ranges.size(); j++)
    {
        // angle_min - start angle of the scan
        // angle_increment - change of angle
        theta = scan->angle_min + (j * scan->angle_increment);
        y = scale * scan->ranges[j] * cos(theta);
        x = scale * scan->ranges[j] * sin(theta);
        y = centerY - y;
        x = centerX - x;

        //fill Mat with data
        if(x>0 && y>0)
        {
            matScan.at<uchar>(y, x) = 255;
        }
    }


    filename_laser = path + "/image_raw_laser/laser_image_" + time_string + ".png";

    string file_laser = "laser_image_" + time_string + ".png";
    myfile << file_laser << "\n";

    try
    {
        imwrite(filename_laser, matScan, compression_params);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("could not save laser image");
    }


    Z = Mat::zeros(matScan.size(), matScan.type());
    Z.copyTo(matScan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "bind_exact");
    ros::NodeHandle nh("~");

    myfile.open(file_path.c_str());

    myfile << "  CarPositionX\tCarPositionY\tCarAngle\tLinearVelocity\tAngularVelocity\tImageName\tLaserName\n";

    message_filters::Subscriber<Image> image_sub(nh, "/camera/rgb/image_raw", 1);
    message_filters::Subscriber<LaserScan> laser_sub(nh, "/scan", 1);
    message_filters::Subscriber<Odometry> odom_sub(nh, "/odom", 1);


    typedef sync_policies::ExactTime<Image, LaserScan, Odometry> MySyncPolicy;
    // ExactTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, laser_sub, odom_sub);
    sync.registerCallback(boost::bind(&bindCallback, _1, _2, _3));

    ros::spin();

    myfile.close();

    return 0;
}
