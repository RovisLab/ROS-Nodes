SOFTWARE
	$sudo apt-get install  ros-indigo-rviz ros-indigo-rqt_reconfigure ros-indigo-freenect-*
	$sudo apt-get install ros-indigo-object-recognition-core
	$sudo apt-get install ros-indigo-object-recognition-*
	$sudo apt-get install -u couchapp

To use on Kinetic, do the following:

mkdir ws && cd ws
wstool init src https://raw.github.com/wg-perception/object_recognition_core/master/doc/source/ork.rosinstall.kinetic.plus
cd src && wstool update -j8
cd .. && rosdep install --from-paths src -i -y
catkin_make
source devel/setup.bash



See the camera registered points in RViz
	$roscore
Hint: Conect the camera to an usb 2.0 port, an 3.0 may cause errors on your virtual machine

	$roslaunch freenect_launch freenect.launch
Hint: You should get some warnings. If not, try unplug the camera usb and plug it back in

	$rosrun rviz rviz 
Set the Fixed Frame (top left of the RViz window) to /camera_depth_optical_frame. Add a PointCloud2 display, and set the topic to /camera/depth/points. Turning the background to light gray can help with viewing. Add an Image display and set the topic to something you would like to see. 
	$rosrun rqt_reconfigure rqt_reconfigure
And select /camera/driver from the drop-down menu. Enable the depth_registration checkbox. Now go back to RViz, and change your PointCloud2 topic to /camera/depth_registered/points. Set Color Transformer to RGB8. You should see a color, 3D point cloud of your scene.
	$ git clone https://github.com/wg-perception/ork_tutorials.git
If you want to see the database with object and meshes 
	$rosrun object_recognition_core push.sh
Hint: it should give you the acces to a local host using couchDB 

Create an object

	$rosrun object_recognition_core object_add.py -n coke -d "A universal can of coke"  --commit
Hint: it should give you an id. Copy it.

Add a mesh for the object
	$rosrun object_recognition_core mesh_add.py YOUR_OBJECT_ID `rospack find object_recognition_tutorials`/data/coke.obj --commit

	Hint: if you can’t find object_recognition_tutorials, try git clone https://github.com/wg-perception/ork_tutorials.git into your workspace

Deleting an object
	$rosrun object_recognition_core object_delete.py OBJECT_ID
 
Object recognition using tabletop

Finding planes

	$rosrun object_recognition_core detection -c `rospack find object_recognition_tabletop` 	conf/detection.table.ros.ork

Then go to RViz graphical window, and add the OrkTable display. Now you should see some planes detected by ORK_Tabletop if your camera is pointing to some plane surfaces.
Finding objects

	$rosrun object_recognition_core detection -c  `rospack find object_recognition_tabletop` 	conf/detection.object.ros.ork

Go back to RViz , and add the OrkObject display. Now if you have a coke can placed on one of the detected planes, ork_tabletop should see it and your beautiful RViz interface should be displaying it

