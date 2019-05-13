#ifndef MA_MPC_DIFF_ROBOT_WRAPPER_H
#define MA_MPC_DIFF_ROBOT_WRAPPER_H

#include <iostream>
#include <map>
#include <math.h>
#include <mutex>

#include "ros/ros.h"
#include "ros/package.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PolygonStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>

#include "PointsSelector.h"

class RobotWrapper
{
public:
    RobotWrapper(const std::map<std::string, std::string> params);
    RobotWrapper(RobotWrapper&& other);

    bool pathComputed() const
    {
        return m_path_computed;
    }

    bool goalReached() const
    {
        return m_goal_reached;
    }

    const nav_msgs::Odometry& getOdom() const
    {
        return m_odom;
    }

    const nav_msgs::Path& getOdomPath() const
    {
        return m_odom_path;
    }

    double getOrientation() const
    {
        return m_orientation;
    }

    void sendCommand(const std::vector<double>& cmd)
    {
        m_twist_msg.linear.y  = 0.;
        m_twist_msg.linear.x  = 0.;
        m_twist_msg.angular.z = 0.;
        if (!m_goal_reached)
        {
            m_twist_msg.linear.x  = cmd[0];
            m_twist_msg.angular.z = cmd[1];
        }

        m_pub_twist.publish(m_twist_msg);
    }

    void publishTrajectory();
    void publishCollisionCircle();

    void setGoalDistance(double dist)
    {
        m_goal_distance = dist;
    }

    void setCollisionCircleRadius(double val)
    {
        m_collision_circle_radius = val;
    }

    double getCollisionCircleRadius() const
    {
        return m_collision_circle_radius;
    }

    PointsSelector<int>::value_type selectPathPoints(double mpc_dist)
    {
        return m_pointsSelector.select(m_odom.pose.pose, mpc_dist);
    }

    std::mutex m;

    std::vector<double> predicted_x;
    std::vector<double> predicted_y;

private:
    ros::NodeHandle m_nh;
    ros::Subscriber m_sub_goal, m_sub_odom, m_sub_path, m_sub_amcl;
    ros::Publisher m_pub_twist, m_pub_mpctraj, m_pub_mpcpoly, m_pub_cc;

    std::string m_robot_name;
    std::string m_map_frame;
    std::string m_odom_frame;
    std::string m_car_frame;
    std::string m_amcl_frame;
    std::string m_globalpath_topic;
    std::string m_cmd_frame;
    std::string m_goal_topic;

    tf::TransformListener m_tf_listener;

    nav_msgs::Path m_odom_path;
    nav_msgs::Odometry m_odom;
    geometry_msgs::Point m_goal_pos;

    PointsSelector<nav_msgs::Path> m_pointsSelector;

    double m_orientation;

    bool m_path_computed;
    bool m_goal_received;
    bool m_goal_reached;

    double m_goal_distance;
    double m_collision_circle_radius;

    geometry_msgs::Twist m_twist_msg;
    std::ofstream m_odomFile;
		std::string package_stringFile = ros::package::getPath("ma_diff_mpc");

    void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
    void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
    void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
    void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
};


#endif
