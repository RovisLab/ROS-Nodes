#!/usr/bin/env python
import argparse
import sys

import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_opencv import highgui

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo

def do_ecto():
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                   depth=ImageBagger(topic_name='/camera/depth/image'),
                   )

    bagreader = ecto_ros.BagReader('Bag Ripper',
                                    baggers=baggers,
                                    bag=sys.argv[1],
                                  )
    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()

    graph = [
                bagreader["image"] >> im2mat_rgb["image"],
                im2mat_rgb["image"] >> highgui.imshow("rgb show", name="rgb")[:],
                bagreader["depth"] >> im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show", name="depth")[:]
            ]

    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)

    sched = ecto.Scheduler(plasm)
    sched.execute()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='Read rgb and depth images from rosbag')
    parser.add_argument('bag_name', type=str, help='ROS Bag name')
    args = parser.parse_args()
    do_ecto()
