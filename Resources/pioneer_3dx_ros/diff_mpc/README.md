# Differential MPC Controller - ROS Node
![Ubuntu version](https://img.shields.io/badge/ubuntu-bionic-orange.svg "Ubuntu version")  ![ROS Version](https://img.shields.io/badge/ros-melodic-green.svg "ROS Version")

This node implement Model Predictive Control (MPC) for a differential robot.

## Node topics
### Input
- **odom_frame** - odometry of the robot (usually *odom*)
- **car_frame** - usually *base_link*
- **amcl_frame** - AMCL frame id (currently *amcl/amcl_pose*)
- **global_path_topic** - global planner path (currently *move_base/NavfnROS/plan*)
- **goal_topic** - goal topic (currently *move_base_simple/goal*)

### Output
- **cmd_frame** - command frame for the robot (currently *cmd_vel*)
- **mpc_traj_frame** - predicted trajectory of the MPC; type: Path (currently *mpc_trajectory*)
- **mpc_poly_frame** - fitted polynomial used for control (currently *mpc_polynomial*)

## Configuration file
The configuration file is located at *param/mpc_params/mpc_params.yaml*. A description for each configuration parameter is present in the same file.

## Steps used to control the robot
1. From the global plan, a set of points are selected, close to the robot, based on the distance required by the MPC
2. Since the command is done in robot coordinate system, the points from global path are rotated and translated.
3. On the points, a polynomial is fitted to smooth the path and also to be able to calculate the orientation in each point using its derivative. The polynomial has always 0 (zero) lateral displacement relative to the robot.
4. The MPC solution is computed using the robot parameters, the predicted state of the robot at the moment of command and the determined polynomial. The solution is: [speed, steering].

## The cost function of the MPC
The cost function is composed based on:
- the difference between the reference velocity and the actual velocity
- the difference between the reference orientation and the actual orientation
- rate of change of velocity

## Learning resources
[Differential robot model](http://planning.cs.uiuc.edu/node659.html)
[Udacity MPC Tutorial source code](https://github.com/udacity/CarND-MPC-Quizzes)
[Hypha-ROS Minicar MPC Example](https://github.com/Hypha-ROS/hypharos_minicar)
