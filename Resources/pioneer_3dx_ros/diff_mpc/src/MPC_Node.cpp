#include <iostream>
#include <map>
#include <math.h>

#include "ros/ros.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <visualization_msgs/Marker.h>

#include "MPC.h"
#include "common.h"

class DiffMPCNode
{
    public:
        DiffMPCNode();

    private:
        ros::NodeHandle m_nh;
        ros::Subscriber m_sub_odom, m_sub_path, m_sub_goal, m_sub_amcl;
        ros::Publisher m_pub_twist, m_pub_mpctraj, m_pub_mpcpoly;

        ros::Timer m_mainLoopTimer;

        tf::TransformListener m_tf_listener;

        geometry_msgs::Point m_goal_pos;
        nav_msgs::Odometry m_odom;
        nav_msgs::Path m_odom_path;
        geometry_msgs::Twist m_twist_msg;

        std::string m_map_frame, m_odom_frame, m_car_frame, m_amcl_frame;

        DiffMPC m_mpc;

        bool m_goal_received, m_goal_reached, m_path_computed, m_debug_info;
        double m_goal_distance;
        double m_wd, m_wr;
        double m_mpc_steps, m_ref_vel, m_mpc_dt;

        void odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg);
        void pathCB(const nav_msgs::Path::ConstPtr& pathMsg);
        void goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg);
        void amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg);
        void controlLoopCB(const ros::TimerEvent&);
}; // DiffMPCNode

DiffMPCNode::DiffMPCNode()
{
    // Private parameters handler
    ros::NodeHandle pn("~");

    // Parameters for Node
    pn.param("debug_info", m_debug_info, false);
    bool publish_est_traj, publish_est_poly;
    pn.param("publish_estimated_trajectory", publish_est_traj, true);
    pn.param("publish_estimated_polynomial", publish_est_poly, true);

    // Parameters for control loop
    double controller_freq = 10.;
    pn.param("controller_freq", controller_freq, 10.);
    pn.param("goal_distance", m_goal_distance, 0.1);

    const double controller_dt = double(1.0 / controller_freq);

    // Robot parameters
    pn.param("wheel_distance", m_wd, 0.3);
    pn.param("wheel_radius", m_wr, 0.09);
    pn.param("reference_velocity", m_ref_vel, 0.7);

    // MPC params
    pn.param("mpc_dt", m_mpc_dt, 0.1);
    pn.param("mpc_steps", m_mpc_steps, 20.);

    // Path and goal topics
    std::string globalpath_topic, goal_topic;
    pn.param<std::string>("global_path_topic", globalpath_topic, "move_base/TrajectoryPlannerROS/global_plan");
    pn.param<std::string>("goal_topic",        goal_topic,       "move_base_simple/goal");

    // Map, odometry amcl and car frame ids
    pn.param<std::string>("map_frame",  m_map_frame,  "map");
    pn.param<std::string>("odom_frame", m_odom_frame, "odom");
    pn.param<std::string>("car_frame",  m_car_frame,  "base_link");
    pn.param<std::string>("amcl_frame", m_amcl_frame, "amcl_pose");

    // Command topic
    std::string cmd_frame;
    pn.param<std::string>("cmd_frame", cmd_frame, "cmd_vel");

    // MPC output topics
    std::string mpc_traj_frame, mpc_poly_frame;
    pn.param<std::string>("mpc_traj_frame", mpc_traj_frame, "mpc_trajectory");
    pn.param<std::string>("mpc_poly_frame", mpc_poly_frame, "mpc_polynomial");

    // Subscribers and publishers
    m_sub_path = m_nh.subscribe(globalpath_topic, 1, &DiffMPCNode::pathCB, this);
    m_sub_goal = m_nh.subscribe(goal_topic,       1, &DiffMPCNode::goalCB, this);
    m_sub_amcl = m_nh.subscribe(m_amcl_frame,     5, &DiffMPCNode::amclCB, this);
    m_sub_odom = m_nh.subscribe(m_odom_frame,     1, &DiffMPCNode::odomCB, this);

    m_pub_twist = m_nh.advertise<geometry_msgs::Twist>(cmd_frame, 1); // command

    if (publish_est_traj)
    {
        ROS_INFO("Publishing estimated trajectory on %s", mpc_traj_frame.c_str());
        m_pub_mpctraj = m_nh.advertise<nav_msgs::Path>(mpc_traj_frame, 1);
    }
    if (publish_est_poly)
    {
        ROS_INFO("Publishing approximated polynomial on %s", mpc_poly_frame.c_str());
        m_pub_mpcpoly = m_nh.advertise<nav_msgs::Path>(mpc_poly_frame, 1);
    }

    m_mainLoopTimer = m_nh.createTimer(ros::Duration(controller_dt), &DiffMPCNode::controlLoopCB, this);

    m_goal_received = false;
    m_goal_reached  = false;
    m_path_computed = false;

#define COPY_MPC_PARAM(name, def) pn.param(name, mpc_parameters[name], def)

    std::map<std::string, double> mpc_parameters;
    COPY_MPC_PARAM("mpc_dt", 0.1);
    COPY_MPC_PARAM("mpc_steps", 20.0);
    COPY_MPC_PARAM("mpc_velocity_weight", 1.);
    COPY_MPC_PARAM("mpc_orientation_weight", 10.);
    COPY_MPC_PARAM("mpc_velocity_change_weight", 1.0);

    COPY_MPC_PARAM("wheel_distance", 0.3);
    COPY_MPC_PARAM("wheel_radius", 0.09);
    COPY_MPC_PARAM("max_velocity", 1.0);
    COPY_MPC_PARAM("reference_velocity", 0.7);

#undef COPY_MPC_PARAM

    m_mpc.LoadParams(mpc_parameters);
}

// Callback: update robot odometry data
void DiffMPCNode::odomCB(const nav_msgs::Odometry::ConstPtr& odomMsg)
{
    m_odom = *odomMsg;
}

// Callback: update reference path
void DiffMPCNode::pathCB(const nav_msgs::Path::ConstPtr& pathMsg)
{
    if(m_goal_received && (m_goal_reached == false))
    {
        try
        {
            m_odom_path.poses.resize(pathMsg->poses.size());
            for (size_t i = 0U; i < pathMsg->poses.size(); ++i)
            {
                m_tf_listener.transformPose(
                    m_nh.resolveName(m_odom_frame),
                    ros::Time(0),
                    pathMsg->poses[i],
                    m_map_frame,
                    m_odom_path.poses[i]);
            }

            m_path_computed = m_odom_path.poses.size() > 0U;
        }
        catch(tf::TransformException &ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

// Callback: Update goal status
void DiffMPCNode::goalCB(const geometry_msgs::PoseStamped::ConstPtr& goalMsg)
{
    ROS_INFO("Goal changed from (%.2lf, %.2lf) to (%.2lf, %.2lf).",
        m_goal_pos.x, m_goal_pos.y,
        goalMsg->pose.position.x, goalMsg->pose.position.y);

    m_goal_pos = goalMsg->pose.position;
    m_goal_received = true;
    m_goal_reached = false;
}

// Callback: Check if the goal was reached by the robot
void DiffMPCNode::amclCB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& amclMsg)
{
    if(m_goal_received)
    {
        const double dx = m_goal_pos.x - amclMsg->pose.pose.position.x;
        const double dy = m_goal_pos.y - amclMsg->pose.pose.position.y;
        const double dist = sqrt(dx*dx + dy*dy);
        if(dist < m_goal_distance)
        {
            m_goal_reached = true;
            m_goal_received = false;
            m_path_computed = false;

            ROS_INFO("The goal has been reached!");
        }
    }
}

template <typename Pose_t>
inline double distance(const Pose_t& pos1, const Pose_t& pos2)
{
    return sqrt(pow(pos1.position.x - pos2.position.x, 2) +
                pow(pos1.position.y - pos2.position.y, 2));
}

void DiffMPCNode::controlLoopCB(const ros::TimerEvent&)
{
    double speed = 0.;
    double steering = 0.;

    if(m_goal_received && (m_goal_reached == false) && m_path_computed)
    {
        // Compute the yaw angle of the robot
        tf::Pose pose;
        tf::poseMsgToTF(m_odom.pose.pose, pose);

        const double px = m_odom.pose.pose.position.x;
        const double py = m_odom.pose.pose.position.y;
        const double th = tf::getYaw(pose.getRotation());
        const double v = m_odom.twist.twist.linear.x;
        const double dth = m_odom.twist.twist.angular.z;

        Eigen::VectorXd state_vec(3); // [x, y, w] = 3
        sRobotState state(state_vec);

        // Kinematic model is used to predict vehicle state at the actual
        // moment of control (current time + delay dt)
        // TBD: Instead of 0.025 should be the time it takes
        // to compute everything and send the command
        state.x  = v * CppAD::cos(0) * 0.025;
        state.y  = v * CppAD::sin(0) * 0.025;
        state.th = dth * 0.025;

        // Determine how many points should be chosen for the reference polynomial
        //const double wp_dist = distance(m_odom_path.poses[0], m_odom_path.poses[1]);
        const double mpc_dist = 1.2 * m_ref_vel * m_mpc_dt * m_mpc_steps;

        // Compute the first index on the path pointsbase_frame_id
        double last_dist = distance(m_odom_path.poses[0].pose, m_odom.pose.pose);
        size_t first_idx = 0U;
        for (size_t i = 1U; i < m_odom_path.poses.size(); ++i)
        {
            const double crt_dist = distance(m_odom_path.poses[i].pose, m_odom.pose.pose);
            if (crt_dist > last_dist)
            {
                first_idx = i;
                break;
            }

            last_dist = crt_dist;
        }

        // Compute the last index on the path points
        size_t last_idx = 0U;
        for (size_t i = first_idx; i < m_odom_path.poses.size(); ++i)
        {
            const double crt_dist = distance(m_odom_path.poses[i].pose, m_odom.pose.pose);
            if (crt_dist > mpc_dist)
            {
                last_idx = i;
                break;
            }
        }

        if (last_idx < first_idx)
        {
            last_idx = m_odom_path.poses.size();
        }

        // Compute the points for the reference path
        // in the robot coordinate system
        const double costh = cos(-th);
        const double sinth = sin(-th);

        const size_t extra_pts = 1U + (last_idx == m_odom_path.poses.size());

        Eigen::VectorXd ref_x(last_idx - first_idx + extra_pts);
        Eigen::VectorXd ref_y(last_idx - first_idx + extra_pts);

        // Predicted robot position
        ref_x[0] = state.x;
        ref_y[0] = state.y;

        for (size_t i = first_idx; i < last_idx; ++i)
        {
            const double dx = m_odom_path.poses[i].pose.position.x - px;
            const double dy = m_odom_path.poses[i].pose.position.y - py;
            ref_x[i - first_idx + 1U] = dx * costh - dy * sinth;
            ref_y[i - first_idx + 1U] = dx * sinth + dy * costh;
        }

        // Goal position
        if (last_idx == m_odom_path.poses.size())
        {
            ref_x[last_idx - first_idx + 1U] = m_goal_pos.x - px;
            ref_y[last_idx - first_idx + 1U] = m_goal_pos.y - py;
        }

        // Fit the polynomial on the points
        const auto coeffs = common::PolyFit(ref_x, ref_y, 4);

        // Solve MPC problem
        // TODO: This does not yet cares about the final orientation
        // given by the goal
        const std::vector<double> mpc_solution = m_mpc.Solve(state_vec, coeffs);

        // Command based on MPC solution
        speed = mpc_solution[0];
        steering = mpc_solution[1];

        if(m_debug_info)
        {
            std::cout << "\n\nDEBUG" << std::endl;
            std::cout << "Current velocity: " << v << std::endl;
            std::cout << "Polynomial coefficients: \n" << coeffs << std::endl;
            std::cout << "Steering angle: \n" << steering << std::endl;
            std::cout << "Speed: \n" << speed << std::endl;
        }

        // Publish approximated polynomial
        if (m_pub_mpcpoly.getTopic().size() > 0U)
        {
            nav_msgs::Path polynomial_path;
            polynomial_path.poses.resize(ref_x.size());
            for (size_t i = 0U; i < ref_x.size(); ++i)
            {
                const double y = common::PolyEval(coeffs, ref_x[i]);

                polynomial_path.poses[i].pose.position.x = ref_x[i];
                polynomial_path.poses[i].pose.position.y = y;
            }

            polynomial_path.header.frame_id = m_nh.resolveName(m_car_frame);
            polynomial_path.header.stamp = ros::Time::now();
            m_pub_mpcpoly.publish(polynomial_path);
        }

        // Publish the mpc trajectory
        if (m_pub_mpctraj.getTopic().size() > 0U)
        {
            nav_msgs::Path mpc_traj;
            mpc_traj.poses.resize(m_mpc.predicted_x.size() - 1U);
            for(size_t i = 0U; i < (m_mpc.predicted_x.size() - 1U); ++i)
            {
                mpc_traj.poses[i].pose.position.x = m_mpc.predicted_x[i];
                mpc_traj.poses[i].pose.position.y = m_mpc.predicted_y[i];
            }

            mpc_traj.header.frame_id = m_nh.resolveName(m_car_frame);
            mpc_traj.header.stamp = ros::Time::now();
            m_pub_mpctraj.publish(mpc_traj);
        }
    }
    else
    {
        if(m_goal_reached && m_goal_received)
        {
            ROS_INFO("The goal has been reached!");
        }
    }

    m_twist_msg.linear.x  = speed;
    m_twist_msg.linear.y  = 0.;
    m_twist_msg.angular.z = steering;
    m_pub_twist.publish(m_twist_msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DiffMPC_Node");
    DiffMPCNode mpc_node;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
