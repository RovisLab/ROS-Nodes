#multi pioneer
  Mutli_pioneer_runfile without any lasers data.
  
  It suppose to make their own path to a point given by us,but it does not work even if we have the map already created!it needs acces to the scan topic in order to use de <<move_base>> package that creates the trajectory path!.it's not like they can give themselfs nav comands to their /cmd_vel topics.

  This runfile exists to show that it does not work withowt laserscan topics for each robot.

If you want to run it:
1.  open a terminal in this folder
2.  insert this command:
      $ source ~/your_workspace/devel/setup.bash
3.  give the runfile run permisions and insert this command:
      $ ./run_multi_pioneer_no_laserscan

Voila!

