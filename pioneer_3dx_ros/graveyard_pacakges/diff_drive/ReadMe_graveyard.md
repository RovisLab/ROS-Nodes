usage:
1.  Run rosaria differential drive:
    $ roslaunch pioneer_description connect_real_pioneer.launch
or
    Spawn a pioneer into the gazebo simulator
  
2.  rosrun diff_drive diff_drive_go_to_goal

Info:
  This will use the diff_drive package to make a robot go to a 2D NAV Goal set in RViz.
  IT does not care of what is near him, so play nice. :P (that's why it is in the graveyard).
