#!/usr/bin/env python
import sys
import argparse

import ecto
import ecto_ros, ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
from ecto_opencv import highgui

ImageSub = ecto_sensor_msgs.Subscriber_Image
CameraInfoSub = ecto_sensor_msgs.Subscriber_CameraInfo
ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo


def do_ecto(bag_name):
    baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_mono'),
                   depth=ImageBagger(topic_name='/camera/depth/image_raw'),
                   )

    bagwriter = ecto_ros.BagWriter('Bag Writer', baggers=baggers,
                                            bag=bag_name,
                                            )
    subs = dict( image=ImageSub(topic_name='/camera/rgb/image_mono',queue_size=0),
                depth=ImageSub(topic_name='/camera/depth/image_raw',queue_size=0),
             )

    sync = ecto_ros.Synchronizer('Synchronizator', subs=subs
                                 )

    im2mat_rgb = ecto_ros.Image2Mat()
    im2mat_depth = ecto_ros.Image2Mat()

    graph = [
                sync["image"] >> im2mat_rgb["image"],
                im2mat_rgb["image"] >> highgui.imshow("rgb show", name="rgb")[:],
                sync[:] >> bagwriter[:],
                sync["depth"] >> im2mat_depth["image"],
                im2mat_depth["image"] >> highgui.imshow("depth show", name="depth")[:]
            ]
    plasm = ecto.Plasm()
    plasm.connect(graph)
    ecto.view_plasm(plasm)

    sched = ecto.Scheduler(plasm)
    sched.execute(niter=30)  # capture a second.

if __name__ == "__main__":

    parser = argparse.ArgumentParser(description='Rosbag rgb and depth images')
    parser.add_argument('bag_name', type=str, default='default.bag', help='ROS Bag name')
    args = parser.parse_args()

    ecto_ros.init(sys.argv, "ecto_node")
    do_ecto(args.bag_name)
