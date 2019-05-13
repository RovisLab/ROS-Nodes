# pioneer_gazebo

  The package contains gazebo worlds and maps available to be used in it's "gazebo_world.lauch" launchfile and map_server package.

<gazebo_world.launch>
    Starts gazebo with the simulated environment.

Args:

~world(string, default: "scout_service")
  
    You can load the world into gazebo.Located in <pioneer_gazebo>/worlds.


    
Example call into a terminal:

  $ roslaunch pioneer_gazebo gazebo_world.launch world:=maze_one
