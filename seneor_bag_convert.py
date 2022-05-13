#!/usr/bin/env python
# -*- coding: utf-8 -*

from lib2to3.pytree import convert
import sys
import progressbar  # pip install progressbar
import numpy as np
import roslib;  # roslib.load_manifest(PKG)
import rosbag
import rospy
import sensor_msgs
import ros_numpy    # sudo apt-get install ros-melodic-ros-numpy
import pcl_msgs     # 
import pcl          # install python-pcl (https://iter01.com/572096.html)
import sensor_msgs.point_cloud2 as pc2
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv_bridge import CvBridgeError


# Reading bag filename from command line or roslaunch parameter.

class BagConverter():

    def __init__(self, bag_file, target_topic, output_dir):
        self.bag_file = bag_file
        self.target_topic = target_topic
        self.output_dir = output_dir
        self.t_start = 0
        self.t_end = 0
        self.topic_list = []
        self.msg_list = []

        print("=== Load Bag info ===")
        print("Bag: " + self.bag_file)

        count = 0
        t_curr = 0
        with rosbag.Bag(bag_file, 'r') as bag:  # ROSbag；
            self.t_start = bag.get_start_time()
            self.t_end = bag.get_end_time()

        print("start time: " + str(self.t_start))
        print("end time: " + str(self.t_end))

    def CompressedImage_convert(self, msg):
        try:
            # cv_image = self.bridge.imgmsg_to_cv2(msg.data, "bgr8")
            np_arr = np.fromstring(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except CvBridgeError as e:
            print
            e
        timestr = "%.9f" % msg.header.stamp.to_sec()

        image_name = timestr + ".png"
        cv2.imwrite(self.output_dir + image_name, cv_image)
        
    def Image_convert(self, msg):
        bridge = CvBridge()
        try:
            cv_image = bridge.imgmsg_to_cv2(msg.data, "bgr8")
        except CvBridgeError as e:
            print
            e
        timestr = "%.9f" % msg.header.stamp.to_sec()

        image_name = timestr + ".png"
        cv2.imwrite(self.output_dir + image_name, cv_image)
        

    def convert_pc_msg_to_np(self, pc_msg):
        # Fix rosbag issues, see: https://github.com/eric-wieser/ros_numpy/issues/23
        pc_msg.__class__ = sensor_msgs.msg._PointCloud2.PointCloud2
        offset_sorted = {f.offset: f for f in pc_msg.fields}
        pc_msg.fields = [f for (_, f) in sorted(offset_sorted.items())]
        
        pc_np = []
        ## By pc2
        for p in pc2.read_points(pc_msg, field_names = ("x", "y", "z", "intensity"), skip_nans=True):
            pc_np.append([p[0], p[1], p[2], p[3]])
        
        pc_pcl = pcl.PointCloud_PointXYZI( np.array(pc_np, dtype=np.float32))

        timestr = "%.9f" % pc_msg.header.stamp.to_sec()
        pc_name = timestr + ".ply"
        pcl.save(pc_pcl, self.output_dir + pc_name, format='PLY', binary=True)
            
    def convert(self):
        #bagFile = input('bag file name:')
        print("=== Start Convert ===")
        print("Topic: " + self.target_topic)
        bar = progressbar.ProgressBar(maxval=(self.t_end - self.t_start), \
        widgets=[progressbar.Bar('=', '[', ']'), ' ', progressbar.Percentage()])
        bar.start()

        with rosbag.Bag(self.bag_file, 'r') as bag:  # ROSbag；
            for topic, msg, t in bag.read_messages():
                if topic == self.target_topic:  # topic；
                    # Check msg
                    if 'CompressedImage' in str(type(msg)):
                        self.CompressedImage_convert(msg)
                    elif 'Image' in str(type(msg)):
                        self.Image_convert(msg)
                    elif 'PointCloud2' in str(type(msg)):
                        self.convert_pc_msg_to_np(msg)
                
                bar.update((t.to_sec() - self.t_start))
        
        bar.finish()


if __name__ == '__main__':
    # rospy.init_node(PKG)
    if len(sys.argv) < 4:
        print("Not enough input argument")
        print(" python sensor_bag_convert.py [bag file] [topic] [output directory]")
        exit()
    elif len(sys.argv) == 4:
        bag_file = sys.argv[1]
        target_topic = sys.argv[2]
        output_dir = sys.argv[3]

    try:
        bag_converter = BagConverter(bag_file, target_topic, output_dir)
        bag_converter.convert()
    except rospy.ROSInterruptException:
        pass
