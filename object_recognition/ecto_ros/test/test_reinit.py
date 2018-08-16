#!/usr/bin/env python
import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_ros_test_utils import *
import os
from catkin.find_in_workspaces import find_in_workspaces
from ecto.opts import run_plasm

ImageSub = ecto_sensor_msgs.Subscriber_Image

def do_ecto(bagname, msg_counts):
    ecto_ros.init(sys.argv, "image_sub_node")
    sub_rgb = ImageSub("image_sub", topic_name='/camera/rgb/image_color', queue_size=0)
    im2mat_rgb = ecto_ros.Image2Mat()
    counter_rgb = ecto.Counter()
    graph = [
                sub_rgb["output"] >> im2mat_rgb["image"],
                im2mat_rgb[:] >> counter_rgb[:],
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    sched = ecto.Scheduler(plasm)
    sched.execute()
    rosbag = play_bag(bagname, delay=0.5)
    wait_bag(rosbag)
    sched.stop()

    print "expecting RGB count:", msg_counts['/camera/rgb/image_color']
    print "RGB count:", counter_rgb.outputs.count
    assert msg_counts['/camera/rgb/image_color'] >= counter_rgb.outputs.count
    assert counter_rgb.outputs.count != 0
    
if __name__ == "__main__":
    bagname = os.path.join(find_in_workspaces(search_dirs=['share'],project='ecto_ros')[0], 'tests', 't01.bag')
    msg_counts = bag_counts(bagname)
    try:
        roscore = start_roscore(delay=1)
        for i in range(1, 10):
            do_ecto(bagname, msg_counts)
    finally:
        roscore.terminate()
