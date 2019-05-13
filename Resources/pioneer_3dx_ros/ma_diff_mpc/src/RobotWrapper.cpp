#include "RobotWrapper.h"
#include <math.h>

RobotWrapper::RobotWrapper(
    const std::map<std::string, std::string> strParams)
    : m_pointsSelector(m_odom_path)
{
    m_path_computed = false;
    m_goal_received = false;
    m_goal_reached = true;

    m_robot_name = strParams.at("robot_name");
    m_map_frame  = strParams.at("map_frame");
    m_odom_frame = strParams.at("odom_frame");
    m_car_frame  = strParams.at("car_frame");
    m_amcl_frame = strParams.at("amcl_frame");
    m_cmd_frame  = strParams.at("cmd_frame");
    m_goal_topic = strParams.at("goal_topic");
    m_globalpath_topic = strParams.at("globalpath_topic");

    // Subscribers and publishers
    m_sub_path = m_nh.subscribe(m_globalpath_topic, 1, &RobotWrapper::pathCB, this);
    m_sub_amcl = m_nh.subscribe(m_amcl_frame, 5, &RobotWrapper::amclCB, this);
    m_sub_odom = m_nh.subscribe(m_odom_frame, 1, &RobotWrapper::odomCB, this);
    m_sub_goal = m_nh.subscribe(m_goal_topic, 1, &RobotWrapper::goalCB, this);

    m_pub_twist = m_nh.advertise<geometry_msgs::Twist>(m_cmd_frame, 1); // command
    m_pub_mpctraj = m_nh.advertise<nav_msgs::Path>(m_robot_name + "/mpc_trajectory", 1);
    m_pub_cc = m_nh.advertise<geometry_msgs::PolygonStamped>(m_robot_name + "/collision_circle", 1);

    std::stringstream ss;
    ss << package_stringFile << "/resources/" << m_robot_name << "_odom.txt";
    new (&m_odomFile) std::ofstream(ss.str());
}

RobotWrapper::RobotWrapper(RobotWrapper&& other)
    : m_pointsSelector(other.m_odom_path)
{
    m_path_computed = other.m_path_computed;
    m_goal_received = other.m_goal_received;
    m_goal_reached  = other.m_goal_reached;

    m_robot_name = other.m_robot_name;
    m_map_frame  = other.m_map_frame;
    m_odom_frame = other.m_odom_frame;
    m_car_frame  = other.m_car_frame;
    m_amcl_frame = other.m_amcl_frame;
    m_cmd_frame  = other.m_cmd_frame;
    m_goal_topic = other.m_goal_topic;
    m_globalpath_topic = other.m_globalpath_topic;

    // Subscribers and publishers
    m_sub_path = m_nh.subscribe(m_globalpath_topic, 1, &RobotWrapper::pathCB, this);
    m_sub_amcl = m_nh.subscribe(m_amcl_frame, 5, &RobotWrapper::amclCB, this);
    m_sub_odom = m_nh.subscribe(m_odom_frame, 1, &RobotWrapper::odomCB, this);
    m_sub_goal = m_nh.subscribe(m_goal_topic, 1, &RobotWrapper::goalCB, this);

    m_pub_twist = m_nh.advertise<geometry_msgs::Twist>(m_cmd_frame, 1); // command
    m_pub_mpctraj = m_nh.advertise<nav_msgs::Path>(m_robot_name + "/mpc_trajectory", 1);
    m_pub_cc = m_nh.advertise<geometry_msgs::PolygonStamped>(m_robot_name + "/collision_circle", 1);

    m_odomFile = std::move(other.m_odomFile);
}

// Callback: update robot odometry data
void RobotWrapper::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    m_odom = *odomMsg;

    // Compute the yaw angle of the robot
    tf::Pose pose;
    tf::poseMsgToTF(m_odom.pose.pose, pose);

    m_orientation = tf::getYaw(pose.getRotation());

    m_odomFile << m_odom.pose.pose.position.x << ' ' << m_odom.pose.pose.position.y;// << ' ';
    //m_odomFile << m_odom.twist.twist.linear.x << ' ' << m_odom.twist.twist.angular.z;
    m_odomFile << std::endl;
}

// Callback: update reference path
void RobotWrapper::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    std::lock_guard<std::mutex> lock(m);
    if(m_goal_received && (m_goal_reached == false))
    {
        std::stringstream ss;
        ss << package_stringFile << "/resources/global_path_" << m_robot_name << ".txt";
        std::ifstream file(ss.str());
        double x, y;
        m_odom_path.poses.clear();
        while (file >> x && file >> y)
        {
            m_odom_path.poses.resize(m_odom_path.poses.size() + 1U);
            m_odom_path.poses.back().pose.position.x = x;
            m_odom_path.poses.back().pose.position.y = y;
        }
        ROS_INFO("%d",m_odom_path.poses.size());

        m_pointsSelector.reset();
        m_path_computed = m_odom_path.poses.size() > 0U;

        // m_odom_path = *pathMsg;
        // m_path_computed = m_odom_path.poses.size() > 0U;
        // m_pointsSelector.reset();
    }
}

// Callback: Check if the goal was reached by the robot
void RobotWrapper::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
    const double dx = m_goal_pos.x - amclMsg->pose.pose.position.x;
    const double dy = m_goal_pos.y - amclMsg->pose.pose.position.y;
    const double dist = sqrt(dx*dx + dy*dy);
    if(dist < m_goal_distance)
    {
        m_goal_reached = true;
        m_goal_received = false;

        m_twist_msg.linear.y  = 0.;
        m_twist_msg.linear.x  = 0.;
        m_twist_msg.angular.z = 0.;
        m_pub_twist.publish(m_twist_msg);

        ROS_INFO("The goal has been reached!");
    }
}

// Callback: Update goal status
void RobotWrapper::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    ROS_INFO("Goal changed from (%.2lf, %.2lf) to (%.2lf, %.2lf).",
        m_goal_pos.x, m_goal_pos.y,
        goalMsg->pose.position.x, goalMsg->pose.position.y);

    m_goal_pos = goalMsg->pose.position;
    m_goal_received = true;
    m_goal_reached = false;
}

void RobotWrapper::publishTrajectory()
{
    // Publish the MPC trajectory
    if (m_pub_mpctraj.getTopic().size() > 0U)
    {
        nav_msgs::Path mpc_traj;
        mpc_traj.poses.resize(predicted_x.size());
        for(size_t i = 0U; i < predicted_x.size(); ++i)
        {
            mpc_traj.poses[i].pose.position.x = predicted_x[i];
            mpc_traj.poses[i].pose.position.y = predicted_y[i];
        }

        mpc_traj.header.frame_id = m_car_frame;
        mpc_traj.header.stamp = ros::Time::now();
        m_pub_mpctraj.publish(mpc_traj);
    }
}

void RobotWrapper::publishCollisionCircle()
{
    // collision circle
    const double alpha = 0.1;
    const size_t NUM_PTS = static_cast<size_t>(2. * M_PI / alpha) + 1U;
    geometry_msgs::PolygonStamped collision_circle;
    collision_circle.polygon.points.resize(NUM_PTS);
    collision_circle.polygon.points[0].x = m_collision_circle_radius;
    collision_circle.polygon.points[0].y = 0.;
    const double cosa = cos(alpha);
    const double sina = sin(alpha);
    for (size_t i = 1U; i < collision_circle.polygon.points.size(); ++i)
    {
        const auto& pos = collision_circle.polygon.points[i - 1U];
        auto& pos2 = collision_circle.polygon.points[i];
        pos2.x = pos.x * cosa - pos.y * sina;
        pos2.y = pos.x * sina + pos.y * cosa;
    }

    collision_circle.header.frame_id = m_car_frame;
    collision_circle.header.stamp = ros::Time::now();
    m_pub_cc.publish(collision_circle);
}
