#include <iostream>
#include <map>
#include <math.h>
#include <mutex>
#include <chrono>

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
#include "RobotWrapper.h"

class MA_DiffMPCNode
{
    public:
        MA_DiffMPCNode();

    private:
        ros::NodeHandle m_nh;

        ros::Timer m_mainLoopTimer;

        geometry_msgs::Point m_goal_pos;

        DiffMPC m_mpc;
        std::vector<RobotWrapper> m_robots;

        bool   m_debug_info;
        double m_wd, m_wr;
        double m_mpc_steps, m_ref_vel, m_mpc_dt;
        double m_computeTime;

        void controlLoopCB(const ros::TimerEvent&);
}; // MA_DiffMPCNode

MA_DiffMPCNode::MA_DiffMPCNode()
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
    double goal_distance = 0.1;
    pn.param("controller_freq", controller_freq, 10.);
    pn.param("goal_distance", goal_distance, 0.1);

    const double controller_dt = double(1.0 / controller_freq);

    // Robot parameters
    double collision_circle_radius = 0.55;
    pn.param("wheel_distance", m_wd, 0.3);
    pn.param("wheel_radius", m_wr, 0.09);
    pn.param("reference_velocity", m_ref_vel, 0.7);
    pn.param("collision_circle_radius", collision_circle_radius, 0.55);

    // MPC params
    pn.param("mpc_dt", m_mpc_dt, 0.1);
    pn.param("mpc_steps", m_mpc_steps, 20.);

    // Path and goal topics
    std::string globalpath_topic, goal_topic;
    pn.param<std::string>("global_path_topic", globalpath_topic, "move_base/TrajectoryPlannerROS/global_plan");
    pn.param<std::string>("goal_topic",        goal_topic,       "move_base_simple/goal");

    // Map, odometry amcl and car frame ids
    std::string map_frame, odom_frame, car_frame, amcl_frame;
    pn.param<std::string>("map_frame",  map_frame,  "map");
    pn.param<std::string>("odom_frame", odom_frame, "odom");
    pn.param<std::string>("car_frame",  car_frame,  "base_link");
    pn.param<std::string>("amcl_frame", amcl_frame, "amcl_pose");

    // Command topic
    std::string cmd_frame;
    pn.param<std::string>("cmd_frame", cmd_frame, "cmd_vel");

    int robots_number = 0;
    pn.param("robots_number", robots_number, 0);

    // Creating the robots
    m_robots.reserve(robots_number);
    for (int i = 0; i < robots_number; ++i)
    {
        std::stringstream ss;
        ss << "pioneer" << (i + 1);
        const std::string robot_name = ss.str();

        std::map<std::string, std::string> strParams;

        strParams["robot_name"] = robot_name;
        strParams["map_frame"]  = map_frame;
        strParams["odom_frame"] = robot_name + "/" + odom_frame;
        strParams["car_frame"]  = robot_name + "/" + car_frame;
        strParams["amcl_frame"] = robot_name + "/" + amcl_frame;
        strParams["cmd_frame"]  = robot_name + "/" + cmd_frame;
        strParams["goal_topic"] = robot_name + "/" + goal_topic;
        strParams["globalpath_topic"] = robot_name + "/" + globalpath_topic;

        m_robots.emplace_back(strParams);
        m_robots.back().setGoalDistance(goal_distance);
        m_robots.back().setCollisionCircleRadius(collision_circle_radius);
    }

    m_mainLoopTimer = m_nh.createTimer(ros::Duration(controller_dt), &MA_DiffMPCNode::controlLoopCB, this);

    m_computeTime = 0.;

#define COPY_MPC_PARAM(name, def) pn.param(name, mpc_parameters[name], def)

    std::map<std::string, double> mpc_parameters;
    COPY_MPC_PARAM("mpc_dt", 0.1);
    COPY_MPC_PARAM("mpc_steps", 20.0);
    COPY_MPC_PARAM("mpc_velocity_weight", 1.);
    COPY_MPC_PARAM("mpc_orientation_weight", 10.);
    COPY_MPC_PARAM("mpc_velocity_change_weight", 1.);
    COPY_MPC_PARAM("mpc_collision_weight", 1.);

    COPY_MPC_PARAM("wheel_distance", 0.3);
    COPY_MPC_PARAM("wheel_radius", 0.09);
    COPY_MPC_PARAM("max_velocity", 1.0);
    COPY_MPC_PARAM("reference_velocity", 0.7);
    COPY_MPC_PARAM("collision_circle_radius", 0.55);

#undef COPY_MPC_PARAM

    m_mpc.LoadParams(mpc_parameters);
}

void MA_DiffMPCNode::controlLoopCB(const ros::TimerEvent&)
{
    const auto start_time = std::chrono::system_clock::now();

    bool path_computed = true;
    for (const auto& robot : m_robots)
    {
        path_computed &= robot.pathComputed();
    }

    bool goal_reached = true;
    for (const auto& robot : m_robots)
    {
        goal_reached &= robot.goalReached();
    }

    if ((goal_reached == false) && path_computed)
    {
        // Determine how many points should be chosen for the reference polynomial
        const double mpc_dist = 1.2 * m_ref_vel * m_mpc_dt * m_mpc_steps;

        std::vector<Eigen::VectorXd> states_vec(m_robots.size());
        std::vector<Eigen::VectorXd> coeffs_vec(m_robots.size());

        for (size_t r = 0U; r < m_robots.size(); ++r)
        {
            // lock the robot until processing is done
            std::lock_guard<std::mutex> lock(m_robots[r].m);

            const auto& m_odom = m_robots[r].getOdom();
            const auto& m_odom_path = m_robots[r].getOdomPath();

            const double px = m_odom.pose.pose.position.x;
            const double py = m_odom.pose.pose.position.y;
            const double th = m_robots[r].getOrientation();
            const double v = m_odom.twist.twist.linear.x;
            const double dth = m_odom.twist.twist.angular.z;


            Eigen::VectorXd& state_vec = states_vec[r];
            state_vec.resize(3); // [x, y, w] = 3
            sRobotState state(state_vec);

            // Kinematic model is used to predict vehicle state at the actual
            // moment of control (current time + delay dt)
            // Delay time is estimated based on last computation time duration
            state.x  = v * CppAD::cos(0.) * m_computeTime;
            state.y  = v * CppAD::sin(0.) * m_computeTime;
            state.th = dth * m_computeTime;

            // Compute the first index on the path points
            const auto pointsRange = m_robots[r].selectPathPoints(mpc_dist);
            const size_t first_idx = pointsRange.first;
            const size_t last_idx = pointsRange.second;

            assert(first_idx != last_idx && "Number of selected points are 0!");

            // THINK: what would happen if the predicted steering would be added to
            // the orientation before coordinate transformation?
            const double sinth = sin(-th);
            const double costh = cos(-th);

            const size_t extra_pts = last_idx == m_odom_path.poses.size();

            Eigen::VectorXd ref_x(last_idx - first_idx + extra_pts);
            Eigen::VectorXd ref_y(last_idx - first_idx + extra_pts);

            for (size_t i = first_idx; i < last_idx; ++i)
            {
                const double dx = m_odom_path.poses[i].pose.position.x - px;
                const double dy = m_odom_path.poses[i].pose.position.y - py;
                ref_x[i - first_idx] = dx * costh - dy * sinth;
                ref_y[i - first_idx] = dx * sinth + dy * costh;
            }

            // Goal position
            if (last_idx == m_odom_path.poses.size())
            {
                ref_x[last_idx - first_idx] = m_goal_pos.x - px;
                ref_y[last_idx - first_idx] = m_goal_pos.y - py;
            }

            // Fit the polynomial on the points
            coeffs_vec[r] = common::PolyFit(ref_x, ref_y, 2);

            // m_robots[r].predicted_x.resize(ref_x.size());
            // m_robots[r].predicted_y.resize(ref_x.size());
            //
            // for (size_t i = 0U; i < ref_x.size(); ++i)
            // {
            //     m_robots[r].predicted_x[i] = ref_x[i];
            //     m_robots[r].predicted_y[i] = common::PolyEval(coeffs_vec[r], ref_x[i]);
            // }
        }

        // Solve MPC problem
        // TODO: This does not yet cares about the final orientation
        // given by the goal
        const auto mpc_solution = m_mpc.Solve(states_vec, coeffs_vec, m_robots);

        // Estimate total computation time from the start of the loop
        // until the command is sent
        const auto end_time = std::chrono::system_clock::now();
        m_computeTime = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time).count() / 1000.;

        // Command based on MPC solution
        for (size_t r = 0U; r < m_robots.size(); ++r)
        {
            m_robots[r].sendCommand(mpc_solution[r]);
        }

        for (auto& robot : m_robots)
        {
            robot.publishTrajectory();
        }
    }
    else
    {
        for (size_t r = 0U; r < m_robots.size(); ++r)
        {
            m_robots[r].sendCommand(std::vector<double>{0., 0.});
        }
    }

    for (size_t r = 0U; r < m_robots.size(); ++r)
    {
        m_robots[r].publishCollisionCircle();
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "DiffMPC_Node");
    MA_DiffMPCNode mpc_node;

    ros::AsyncSpinner spinner(1);
    spinner.start();
    ros::waitForShutdown();

    return 0;
}
