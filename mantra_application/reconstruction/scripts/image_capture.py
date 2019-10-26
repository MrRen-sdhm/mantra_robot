#!/usr/bin/env python
# -*- coding: utf-8 -*-

#system
import rospy
import os
import sys
import time
import subprocess
import copy
import numpy as np
import cv2
import shutil


# ply reader
from plyfile import PlyData, PlyElement

# ROS
import tf
import tf2_ros
import rospy
import rosbag
import sensor_msgs
from cv_bridge import CvBridge

# Utils
import utils as Utils
import ros_utils as rosUtils



ROS_BAGGING_NODE_NAME = "spartan_rosbag_node"
        

class TFWrapper(object):

    def __init__(self):
        self.tfBuffer = None
        self.tfListener = None
        self.setup()

    def setup(self):
        self.tfBuffer = tf2_ros.Buffer()
        self.tfListener = tf2_ros.TransformListener(self.tfBuffer)

    def getBuffer(self):
        while self.tfBuffer is None:
            time.sleep(0.1)

        return self.tfBuffer



class ImageCapture(object):
    """
    Class used to capture synchronized images. It can also read them from a log
    """

    def __init__(self, rgb_topic, depth_topic, camera_info_topic,
        camera_frame, world_frame, rgb_encoding='bgr8'):

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tfBuffer = None    

        self.rgb_encoding = rgb_encoding
        self.topics_dict = dict()
        self.topics_dict['rgb'] = rgb_topic
        self.topics_dict['depth'] = depth_topic
        print "topics_dict:", self.topics_dict
        self.camera_info_topic = camera_info_topic

        self.cv_bridge = CvBridge()

    def resetCache(self):
        self.depth_msgs = []
        self.rgb_msgs = []
        self.camera_to_base_transforms = []

    def setupTF(self):
        tfWrapper = TFWrapper()
        tfBuffer = tfWrapper.getBuffer()
        if self.tfBuffer is None:
            self.tfBuffer = tfBuffer

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def start(self):
        self.resetCache()
        self.setupSubscribers()
        self.setupTF()

    def getRGBOpticalFrameToWorldTransform(self, ros_time=None):
        """ Get transform from tfBuffer at a time stamp. """
        if ros_time is None:
            ros_time = rospy.Time(0)
        while not rospy.is_shutdown():
            try:
                rgbOpticalFrameToWorld = self.tfBuffer.lookup_transform(self.world_frame, self.camera_frame, ros_time)
                return rgbOpticalFrameToWorld
                break
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

    def setupSubscribers(self):
        self.subscribers = {}
        self.subscribers['rgb'] = rospy.Subscriber(self.topics_dict['rgb'], sensor_msgs.msg.Image, self.onRgbImage)
        self.subscribers['depth'] = rospy.Subscriber(self.topics_dict['depth'], sensor_msgs.msg.Image, self.onDepthImage)

    def onRgbImage(self, msg):
        print "get rgb"
        self.rgb_msgs.append(msg)

    def onDepthImage(self, msg):
        print "get depth"
        self.depth_msgs.append(msg)

    def getTransforms(self):
        for msg in self.depth_msgs:
            stamp = msg.header.stamp
            camera_to_world = self.getRGBOpticalFrameToWorldTransform(ros_time=stamp)            
            print "\n\ncamera_to_world:\n", camera_to_world
            self.camera_to_base_transforms.append(camera_to_world)

    def stop(self):
        self.subscribers['rgb'].unregister()
        self.subscribers['depth'].unregister()

    def synchronize_rgb_and_depth_msgs(self):
        self.rgb_timestamps = []
        for msg in self.rgb_msgs:
            stamp = msg.header.stamp
            print type(stamp)
            # self.rgb_timestamps.append()

    def load_ros_bag(self, ros_bag_filename):
        self.ros_bag = rosbag.Bag(ros_bag_filename, "r")

    def process_ros_bag(self, bag, output_dir, rgb_only=False):

        image_topics = []
        for key, topic in self.topics_dict.iteritems():
            image_topics.append(topic)


        # load all the images
        rgb_data = dict()
        rgb_data['msgs'] = []
        rgb_data['cv_img'] = []
        rgb_data['timestamps'] = []



        depth_data = dict()
        depth_data['msgs'] = []
        depth_data['cv_img'] = []
        depth_data['timestamps'] = []
        depth_data['camera_to_world'] = []

        print "image_topics: ", image_topics

        # extract TF information
        tf_t = rosUtils.setup_tf_transformer_from_ros_bag(bag, cache_time_secs=3600)

        log_rate = 100

        counter = 0
        for topic, msg, t in bag.read_messages(topics=image_topics):
            counter += 1

            # skip the first 30 images due to transform errors . . . 
            # if counter < 30:
            #     continue

            if counter % log_rate == 0:
                print "processing image message %d" %(counter)


            data = None
            if "rgb" in topic:
                cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding=self.rgb_encoding)
                data = rgb_data
            elif "depth" in topic:
                cv_img = rosUtils.depth_image_to_cv2_uint16(msg, bridge=self.cv_bridge)
                data = depth_data

                try:
                    # rot ix (x,y,z,w)
                    (trans, rot) = tf_t.lookupTransform(self.world_frame, self.camera_frame, msg.header.stamp)
                except:
                    print "wasn't able to get transform for image message %d, skipping" %(counter)
                    continue


                depth_data['camera_to_world'].append((trans, rot))
                

            # save the relevant data
            data['msgs'].append(msg)
            data['cv_img'].append(cv_img)
            data['timestamps'].append(msg.header.stamp.to_nsec())

        print "Extracted %d rgb images" %(len(rgb_data['msgs']))
        print "Extracted %d depth images" %(len(depth_data['msgs']))
        

        rgb_data['timestamps'] = np.array(rgb_data['timestamps'])

        # synchronize the images
        synchronized_rgb_imgs = []
        for idx, stamp in enumerate(depth_data['timestamps']):
            rgb_idx = ImageCapture.lookup_synchronized_image(stamp, rgb_data['timestamps'])
            synchronized_rgb_imgs.append(rgb_data['cv_img'][rgb_idx])
            if idx % log_rate == 0:
                print "depth image %d matched to rgb image %d" %(idx, rgb_idx)


        # save to a file
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)


        pose_data = dict()

        for idx, depth_img in enumerate(depth_data['cv_img']):
            rgb_img = synchronized_rgb_imgs[idx]

            rgb_filename = "%06i_%s.png" % (idx, "rgb")
            rgb_filename_full = os.path.join(output_dir, rgb_filename)

            depth_filename = "%06i_%s.png" % (idx, "depth")
            depth_filename_full = os.path.join(output_dir, depth_filename)

            if idx % log_rate == 0:
                print "writing image %d to file %s" %(idx, rgb_filename)
            
            cv2.imwrite(rgb_filename_full, rgb_img)
            if not rgb_only:
                cv2.imwrite(depth_filename_full, depth_img)

            pose_data[idx] = dict()
            d = pose_data[idx] 
            trans, rot = depth_data['camera_to_world'][idx]
            quat_wxyz = [rot[3], rot[0], rot[1], rot[2]]
            transform_dict = Utils.dictFromPosQuat(trans, quat_wxyz)
            d['camera_to_world'] = transform_dict
            d['timestamp'] = depth_data['timestamps'][idx]
            d['rgb_image_filename'] = rgb_filename
            d['depth_image_filename'] = depth_filename



        Utils.saveToYaml(pose_data, os.path.join(output_dir,'pose_data.yaml'))

        # extract the camera info msg

        camera_info_msg = None
        for topic, msg, t in bag.read_messages(topics=self.camera_info_topic):
            camera_info_msg = msg
            break

        
        camera_info_dict = rosUtils.camera_info_dict_from_camera_info_msg(camera_info_msg)
        
        # NOTE: currently the batch_extract_and_fuse_all_scenes.py
        # script checks for the existence of this file (camera_info.yaml) 
        # to determine if the extraction process was completed.

        Utils.saveToYaml(camera_info_dict, os.path.join(output_dir,'camera_info.yaml'))
        
    @staticmethod
    def lookup_synchronized_image(query_time, timestamps):
        """
        Parameters:
            query_time: int
                the time you want to find closest match to
            

        """
        idx = np.searchsorted(timestamps, query_time)
        return min(idx, np.size(timestamps)-1)


def main():
    rgb_topic = "/camera/color/image_raw"
    depth_topic = "/camera/aligned_depth_to_color/image_raw"
    camera_info_topic = "/camera/color/camera_info"
    rgb_encoding = 'bgr8'

    camera_frame = "camera_color_optical_frame"
    # camera_frame = "camera_link"
    world_frame = "base_link"
    # world_frame = "camera_color_frame"

    ros_bag_filename = "/home/sdhm/image_capture_test.bag"
    output_dir = "/home/sdhm/extracted_images"
    
    image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
        camera_frame, world_frame, rgb_encoding=rgb_encoding)


    # image_capture.load_ros_bag(ros_bag_filename)
    # image_capture.process_ros_bag(image_capture.ros_bag, output_dir)

    image_capture.start() # start image subscriber
    time.sleep(2)
    image_capture.stop() # stop image subscriber
    image_capture.getTransforms() # get tf from tfBuffer at a time stamp 
    num_depth_images = len(image_capture.depth_msgs)
    num_rgb_images = len(image_capture.rgb_msgs)
    print "received %d depth images" %(num_depth_images)
    print "received %d rgb images" %(num_rgb_images)

    # image_capture.synchronize_rgb_and_depth_msgs()


if __name__ == "__main__":
    rospy.init_node("image_capture_test")
    main()