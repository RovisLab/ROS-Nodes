//ROS
#include "ros/ros.h"
#include <ros/package.h>

#include "sensor_msgs/LaserScan.h"
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <nav_msgs/Odometry.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>
#include <string>
#include <sys/types.h>
#include <sys/stat.h>


using namespace std;
using namespace cv;
using namespace sensor_msgs;
using namespace nav_msgs;
using namespace message_filters;

const int width = 240;
const int height = 320;
const int centerX = width/2;
const int centerY = height/2;
int scale = 40;
float theta;
int y, x;
Mat matScan(width, height, CV_8UC1);
Mat matScanRGB(width, height, CV_8UC3);
Mat Z, result;
int rows, cols;

string package_string_path= ros::package::getPath("data_saver");
string image_path, robot_namespace;
string robot_path;

ofstream myfile;

void bindCallback( const LaserScanConstPtr& scan, const ImageConstPtr& image, const OdometryConstPtr& odom )
{
    myfile << odom->pose.pose.position.x << "\t" << odom->pose.pose.position.y << "\t" << odom->pose.pose.orientation.w << "\t" << odom->twist.twist.linear.x << "\t" << odom->twist.twist.angular.z << "\t";

    cv_bridge::CvImagePtr cv_ptr;

    string stamp_string = boost::lexical_cast<std::string>(scan->header.stamp);

    image_path = robot_path + "/image_" +stamp_string + ".png";

    string file_image = "image_" + stamp_string + ".png";

    myfile << file_image << "\n";

    vector<int> compression_params;
    compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
    compression_params.push_back(5);

    //laser data acquisition
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

    try
    {
        cv_ptr = cv_bridge::toCvCopy(image, sensor_msgs::image_encodings::RGB8);
        resize(cv_ptr->image, cv_ptr->image, Size(height, width));
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("Could not convert from '%s' to 'rgb8'.", image->encoding.c_str());
    }

    cvtColor(matScan, matScanRGB, COLOR_GRAY2BGR);

    rows = matScan.rows;
    cols = matScan.cols + cv_ptr->image.cols;

    result.create(rows, cols, CV_8UC3);
    hconcat(cv_ptr->image, matScanRGB, result);

    try
    {
        imwrite(image_path, result, compression_params);
    }
    catch(cv_bridge::Exception& e)
    {
        ROS_ERROR("could not save the image");
    }

    Z = Mat::zeros(matScan.size(), matScan.type());
    Z.copyTo(matScan);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "state_saver");
    ros::NodeHandle nh("~");

    if (!nh.getParam("robot_namespace", robot_namespace))
        robot_namespace = "pioneer1";

    int status;

    robot_path = package_string_path + "/" + robot_namespace;

    status = mkdir(robot_path.c_str(), S_IRWXU | S_IRWXG | S_IROTH | S_IXOTH);

    string state_path_filename = robot_path + "/state_run.txt";

    myfile.open(state_path_filename.c_str());

    myfile << "CarPositionX\tCarPositionY\tCarAngle\tLinearVelocity\tAngularVelocity\tImageName\n";

    message_filters::Subscriber<Image> image_sub(nh, "/" + robot_namespace + "/camera/rgb/image_raw", 100);
    message_filters::Subscriber<LaserScan> laser_sub(nh, "/" + robot_namespace + "/scan", 10);
    message_filters::Subscriber<Odometry> odom_sub(nh, "/" + robot_namespace + "/odom", 100);

    typedef sync_policies::ApproximateTime<LaserScan, Image, Odometry> MySyncPolicy;
    // ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
    Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), laser_sub, image_sub, odom_sub);
    sync.registerCallback(boost::bind(&bindCallback, _1, _2, _3));

    ros::spin();

    myfile.close();

    return 0;
}
