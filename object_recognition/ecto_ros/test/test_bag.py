#!/usr/bin/env python
from ecto_ros_test_utils import *
import ecto
from ecto_ros import BagReader, Image2Mat
import ecto_ros.ecto_sensor_msgs as ecto_sensor_msgs
import os
import unittest
import rospkg
from catkin.find_in_workspaces import find_in_workspaces

ImageBagger = ecto_sensor_msgs.Bagger_Image
CameraInfoBagger = ecto_sensor_msgs.Bagger_CameraInfo


class TestBag(unittest.TestCase):

    def test_bag(self):
        bagname = os.path.join(find_in_workspaces(search_dirs=['share'], project='ecto_ros')[0], 'tests', 't01.bag')
        counts = bag_counts(bagname)

        #test that the counts are the same.
        print counts
        self.assertEqual(counts['/camera/rgb/image_color'], counts['/camera/depth/image'])
        self.assertNotEqual(counts['/camera/rgb/image_color'], 0)

        baggers = dict(image=ImageBagger(topic_name='/camera/rgb/image_color'),
                    depth=ImageBagger(topic_name='/camera/depth/image'),
                    )

        bagreader = BagReader('Bag Ripper',
                                      baggers=baggers,
                                      bag=bagname,
                                    )
        im2mat_rgb = Image2Mat()
        im2mat_depth = Image2Mat()
        counter_rgb = ecto.Counter()
        counter_depth = ecto.Counter()

        graph = [
                  bagreader["image"] >> im2mat_rgb["image"],
                  bagreader["depth"] >> im2mat_depth["image"],
                  im2mat_rgb[:] >> counter_rgb[:],
                  im2mat_depth[:] >> counter_depth[:]

              ]

        plasm = ecto.Plasm()
        plasm.connect(graph)
        #ecto.view_plasm(plasm)

        plasm.execute(niter=0)
        print "expecting count:", counts['/camera/rgb/image_color']
        print "RGB count:", counter_rgb.outputs.count
        print "Depth count:", counter_depth.outputs.count
        assert counts['/camera/rgb/image_color'] == counter_rgb.outputs.count
        assert counts['/camera/depth/image'] == counter_depth.outputs.count

if __name__ == "__main__":
    import rosunit
    rosunit.rosrun('ecto_ros', 'test_bag', TestBag)
