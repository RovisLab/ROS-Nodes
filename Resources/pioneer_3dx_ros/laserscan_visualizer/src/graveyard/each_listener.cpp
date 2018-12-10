//ROS
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include <ros/package.h>
#include <nav_msgs/Odometry.h>

#include <fstream>

using namespace std;
using namespace cv;

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
string file_path = path + "/data_text.txt";

ofstream myfile;



void imageCallback(const sensor_msgs::Image::ConstPtr& image)
{
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



    //ROS_INFO("pack path: %s", path.c_str());

    ros::Time time_image = image->header.stamp;

    boost::posix_time::ptime my_posix_time_image = time_image.toBoost();

    std::string iso_time_image_str = boost::posix_time::to_iso_extended_string(my_posix_time_image);

    filename_image = path + "/image_raw/image." + iso_time_image_str + ".png";

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5);

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        resize(cv_ptr->image, cv_ptr->image, Size(height, width));
        imwrite(filename_image, cv_ptr->image, compression_params);
        //ROS_INFO("filename: %s", filename.c_str());
        //imshow("image", cv_ptr->image);
        //waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'bgr8'.", image->encoding.c_str());
    }

}

void laserscanCallback( const sensor_msgs::LaserScan::ConstPtr& scan )
{
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

    ros::Time time_laser = scan->header.stamp;

    boost::posix_time::ptime my_posix_time_laser = time_laser.toBoost();

    std::string iso_time_laser_str = boost::posix_time::to_iso_extended_string(my_posix_time_laser);

    filename_laser = path + "/image_raw/laser." + iso_time_laser_str + ".png";

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5);

    try
    {
        imwrite(filename_laser, matScan, compression_params);
        //imshow("image", matScan);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("could not save laser image");
    }
    Z = Mat::zeros(matScan.size(), matScan.type());
    Z.copyTo(matScan);
}
void odomCallback( const nav_msgs::Odometry::ConstPtr& odom)
{
    /*
    ROS_INFO("Seq: [%d]", odom->header.seq);
    ROS_INFO("Position-> x: [%f], y: [%f], z: [%f]", odom->pose.pose.position.x,odom->pose.pose.position.y, odom->pose.pose.position.z);
    ROS_INFO("Orientation-> x: [%f], y: [%f], z: [%f], w: [%f]", odom->pose.pose.orientation.x, odom->pose.pose.orientation.y, odom->pose.pose.orientation.z, odom->pose.pose.orientation.w);
    ROS_INFO("Vel-> Linear: [%f], Angular: [%f]", odom->twist.twist.linear.x,odom->twist.twist.angular.z);
    */
    myfile << odom->pose.pose.position.x << "\t" << odom->pose.pose.position.y << "\t" << odom->pose.pose.orientation.w << "\t" << odom->twist.twist.linear.x << "\t" << odom->twist.twist.angular.z << "\n";
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "each_visualizer");
    ros::NodeHandle nh("~");
    //namedWindow("image", WINDOW_AUTOSIZE);
    myfile.open(file_path.c_str());

    myfile << "CarPositionX\tCarPositionY\tCarAngle\tLinearVelocity\tAngularVelocity\n";

    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub = it.subscribe("/camera/rgb/image_raw", 1, imageCallback);

    ros::Subscriber laser_sub = nh.subscribe("/scan", 1, laserscanCallback);
    ros::Subscriber odom_sub = nh.subscribe("/odom", 1, odomCallback);

    ros::spin();
    myfile.close();
    return 0;
}
