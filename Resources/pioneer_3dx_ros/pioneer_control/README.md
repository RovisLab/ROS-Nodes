# pioneer_control

Turtelbot_teleop node remapped in order to control the pioneer's movement(/cmd_vel topic).It moves on the x linear velocity and z angular velocity.

  <pioneer_teleop.launch>
    Takes input controls from the keyboard and publish velocity commands to the command velocity topic.

Params:
  
~scale_linear(double, default: 1)
  
    How much to scale the linear velocity.

~scale_angular(double, default: 1)

    How much to scale the angular velocity.

Topics:

~pioneer1/cmd_vel
  
    The pioneer model takes movement command from this topic.

Example usage:
  
  $ roslaunch pioneer_control pioneer_teleop.launch

    <pioneer_ns_teleop.launch>
        Modified the first launchfile so we can teleoperate a specific robot in the multi spawn scenario.

Args:

~robot_name(string, default: "pioneer1")
  
    The prefix of the topic and node name of an individual robot.

Example usage:

  $ roslaunch pioneer_control pioneer_ns_teleop robot_name:=pioneer2

    <pioneer_control.launch>
      Controls the robot's joints and transforms them to tf in order to visualize them on rviz.Uses <control_manager> package to control the joints and <robot_state_publisher> package to transform the joins for visualization.
      NOTE:outdated, I don't call these nodes from here.

    <pioneer_joy_teleop.launch>
          Allows to teleoperate a real Pioneer mobile robot using a Logitech wireless F710 gamepad.
Args:

~robot_name(string, default: "pioneer1")

Example usage :
  
  $ roslaunch pioneer_control pioneer_joy_teleop.launch robot_name:=pioneer2
Attention!launchfile still under developement.
