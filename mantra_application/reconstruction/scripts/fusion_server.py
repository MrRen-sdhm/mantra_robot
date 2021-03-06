#!/usr/bin/env python
#coding:utf-8

# system
import os
import time
import subprocess
import multiprocessing
import numpy as np
import cv2
import shutil
import yaml
import glob

# ROS
import tf2_ros
import rospy
import rosbag
import sensor_msgs
from cv_bridge import CvBridge

import utils as utils
import ros_utils as ros_utils

from transforms3d import quaternions
from mantra_application.srv import *
from get_camera_pose_image_manual import *


class RobotMove(object):
    def __init__(self):
        pass

    @staticmethod
    def move_to_pose_named(pose_name):
        print "[INFO] Wait for service..."
        rospy.wait_for_service('move_to_pose_named')
        try:
            move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
            resp = move_to_pose_named(pose_name)
            if resp.success:
                print "[INFO] Robot move to pose named %s success." % pose_name
                return True
        except rospy.ServiceException, e:
            print "[ERROR] Robot move service call failed: %s" % e

    @staticmethod
    def move_to_poses_named(pose_names):
        print "[INFO] Wait for service..."
        rospy.wait_for_service('move_to_poses_named')
        try:
            move_to_poses_named = rospy.ServiceProxy('move_to_poses_named', MoveToPosesNamed)
            resp = move_to_poses_named(pose_names)
            if resp.success:
                print "[INFO] Robot move to poses named ", pose_names, "success."
                return True
        except rospy.ServiceException, e:
            print "[ERROR] Robot move service call failed: %s" % e

    @staticmethod
    def set_vel_scaling(scale, test=False):
        print("[SRVICE] Wait for service ...")
        try:
            rospy.wait_for_service('set_vel_scaling', timeout=5)
            print("[SRVICE] Found set vel scaling service!")
        except rospy.ROSException:
            print("[ERROR] Set vel scaling service did not started!")
            return False

        if test:  # test success
            return True

        try:
            set_vel_scaling = rospy.ServiceProxy('set_vel_scaling', SetVelScaling)
            resp = set_vel_scaling(scale)
            return resp.success
        except rospy.ServiceException, e:
            print("[SRVICE] Set vel scaling service call failed: %s" % e)
            return False


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
                 camera_frame, world_frame):

        self.camera_frame = camera_frame
        self.world_frame = world_frame
        self.tfBuffer = None

        self.topics_dict = dict()
        self.topics_dict['rgb'] = rgb_topic
        self.topics_dict['depth'] = depth_topic
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
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

    def setupSubscribers(self):
        self.subscribers = {'rgb': rospy.Subscriber(self.topics_dict['rgb'], sensor_msgs.msg.Image, self.onRgbImage),
                            'depth': rospy.Subscriber(self.topics_dict['depth'], sensor_msgs.msg.Image,
                                                      self.onDepthImage)}

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

    def process_ros_bag(self, bag, output_dir, rgb_only=False, stamp_ls=None):
        if stamp_ls is None:
            stamp_ls = []

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

        # setup transformer to extract TF information
        tf_t = ros_utils.setup_tf_transformer_from_ros_bag(bag, cache_time_secs=3600, verbose=False)

        # ###############################   Extract images and tf   #################################
        log_rate = 10
        counter = 0
        for topic, msg, t in bag.read_messages(topics=image_topics):
            counter += 1

            # skip the first 30 images due to transform errors . . .
            # if counter < 30:
            #     continue

            if counter % log_rate == 0:
                print "processing image message %d" % counter

            data = None
            cv_img = None
            if "color" in topic and "depth" not in topic:
                cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
                data = rgb_data
            elif "depth" in topic:
                cv_img = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='16UC1')
                data = depth_data

                # look up stamped transform between world_frame and camera_frame
                try:
                    (trans, rot) = tf_t.lookupTransform(self.world_frame, self.camera_frame, msg.header.stamp)
                except tf2_ros.ExtrapolationException, e:
                    print "[Exception] tf2_ros.ExtrapolationException:\n", e
                    print "wasn't able to get transform for image message %d, skipping" % counter
                    continue
                # except tf2_ros.LookupException, e:
                #     print "[Exception] tf2_ros.LookupException:\n", e
                #     print "wasn't able to get transform for image message %d, skipping" % counter
                #     # continue
                #     return False

                depth_data['camera_to_world'].append((trans, rot))

            # save the relevant data
            data['msgs'].append(msg)
            data['cv_img'].append(cv_img)
            data['timestamps'].append(msg.header.stamp.to_nsec())

        print "Extracted %d rgb images" % (len(rgb_data['msgs']))
        print "Extracted %d depth images" % (len(depth_data['msgs']))

        # ###############################   Get synchronized images   #################################
        rgb_data['timestamps'] = np.array(rgb_data['timestamps'])

        synchronized_rgb_imgs = []
        for idx, stamp in enumerate(depth_data['timestamps']):
            rgb_idx = ImageCapture.lookup_synchronized_image(stamp, rgb_data['timestamps'])
            synchronized_rgb_imgs.append(rgb_data['cv_img'][rgb_idx])
            if idx % log_rate == 0:
                print "depth image %d matched to rgb image %d" % (idx, rgb_idx)

        print "synchronized rgb imgs num:", len(synchronized_rgb_imgs)

        # ######################   Save synchronized images and camera pose  #########################
        if not os.path.isdir(output_dir):
            os.makedirs(output_dir)

        pose_data = dict()
        for idx, depth_img in enumerate(depth_data['cv_img']):
            rgb_img = synchronized_rgb_imgs[idx]

            # rgb_filename = "%06i_%s.png" % (idx, "rgb")
            rgb_filename = "%06i_%s.jpg" % (idx, "rgb")
            rgb_filename_full = os.path.join(output_dir, rgb_filename)

            depth_filename = "%06i_%s.png" % (idx, "depth")
            depth_filename_full = os.path.join(output_dir, depth_filename)

            if idx % log_rate == 0:
                print "writing image %d to file %s" % (idx, rgb_filename)

            # cv2.imwrite(rgb_filename_full, rgb_img, [int(cv2.IMWRITE_PNG_COMPRESSION), 100])
            cv2.imwrite(rgb_filename_full, rgb_img, [int(cv2.IMWRITE_JPEG_QUALITY), 100])

            if not rgb_only:
                cv2.imwrite(depth_filename_full, depth_img, [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

            pose_data[idx] = dict()
            d = pose_data[idx]
            trans, rot = depth_data['camera_to_world'][idx]
            quat_wxyz = [rot[3], rot[0], rot[1], rot[2]]
            transform_dict = utils.dictFromPosQuat(trans, quat_wxyz)
            d['camera_to_world'] = transform_dict
            d['timestamp'] = depth_data['timestamps'][idx]
            d['rgb_image_filename'] = rgb_filename
            d['depth_image_filename'] = depth_filename

        utils.saveToYaml(pose_data, os.path.join(output_dir, 'pose_data.yaml'))
        # extract the camera info msg

        camera_info_msg = None
        for topic, msg, t in bag.read_messages(topics=self.camera_info_topic):
            camera_info_msg = msg
            break

        camera_info_dict = ros_utils.camera_info_dict_from_camera_info_msg(camera_info_msg)
        utils.saveToYaml(camera_info_dict, os.path.join(output_dir, 'camera_info.yaml'))

        # #############################      Save stamped images      ################################
        pose_data = dict()
        stamped_img_output_dir = os.path.join(os.path.dirname(output_dir), "images_stamped")
        if len(stamp_ls) > 0:
            for idx, stamp in enumerate(stamp_ls):
                # get index
                rgb_idx = ImageCapture.lookup_synchronized_image(stamp[1], rgb_data['timestamps'])
                depth_idx = ImageCapture.lookup_synchronized_image(stamp[1], depth_data['timestamps'])

                # save images
                if not os.path.isdir(stamped_img_output_dir):
                    os.makedirs(stamped_img_output_dir)

                rgb_filename = "%s_%s.jpg" % (stamp[0], "rgb")
                rgb_filename_full = os.path.join(stamped_img_output_dir, rgb_filename)

                depth_filename = "%s_%s.png" % (stamp[0], "depth")
                depth_filename_full = os.path.join(stamped_img_output_dir, depth_filename)
                cv2.imwrite(rgb_filename_full, rgb_data['cv_img'][rgb_idx], [int(cv2.IMWRITE_JPEG_QUALITY), 100])
                cv2.imwrite(depth_filename_full, depth_data['cv_img'][depth_idx], [int(cv2.IMWRITE_PNG_COMPRESSION), 0])

                # print "[DEBUG] stamped pose:%s rgb_idx:" % (stamp[0], rgb_idx)
                # print "[DEBUG] stamped pose:%s depth_idx:" % (stamp[0], depth_idx)

                pose_data[idx] = dict()
                d = pose_data[idx]
                trans, rot = depth_data['camera_to_world'][depth_idx]
                quat_wxyz = [rot[3], rot[0], rot[1], rot[2]]
                transform_dict = utils.dictFromPosQuat(trans, quat_wxyz)
                d['camera_to_world'] = transform_dict
                d['timestamp'] = depth_data['timestamps'][depth_idx]
                d['rgb_image_filename'] = rgb_filename
                d['depth_image_filename'] = depth_filename

            utils.saveToYaml(pose_data, os.path.join(stamped_img_output_dir, 'pose_data.yaml'))
            utils.saveToYaml(camera_info_dict, os.path.join(stamped_img_output_dir, 'camera_info.yaml'))

    @staticmethod
    def lookup_synchronized_image(query_time, timestamps):
        """
        Parameters:
            query_time: int
                the time you want to find closest match to
        """
        idx = np.searchsorted(timestamps, query_time)
        return min(idx, np.size(timestamps) - 1)


class FusionServer(object):

    def __init__(self):
        self.bagging = False
        self.rosbag_proc = None
        self.tfBuffer = None
        self.robot_move = RobotMove()
        self.setupConfig()
        self.setupTF()
        self.stamp_ls = []
        self.im_converter = image_converter()

    def setupConfig(self):
        self.config = dict()
        self.config['world_frame'] = "base_link"
        self.config['camera_frame'] = "camera_color_optical_frame"
        self.config['sleep_time_before_bagging'] = 2.0
        self.config['bag_folder'] = "data"
        # self.config['bag_folder'] = "data/muti_object"
        self.config['obj_name'] = "multi"  # "bread" "yogurt" "banana" "charger" "mango" "lemon"

        self.config['keep_raw_bag'] = True
        self.config['keep_raw_images'] = False

        # if use close up poses to create point clouds
        self.config['use_close_up_poses'] = False  # Mark
        # self.config['use_close_up_poses'] = True  # Mark

        self.config['fusion_voxel_size'] = 0.002
        time.sleep(5)

        # self.config['work_space'] = [-4.0, 4.0, -4.0, 4.0, -4.0, 4.0]  # workspace to create one view cloud [-4.0, 4.0, -4.0, 4.0, -4.0, 4.0]
        # self.config['work_space'] = [0.4, 2.0, -0.4, 0.4, -0.01, 1.0]  # workspace to create one view cloud [0.4, 2.0, -0.4, 0.4, -0.01, 1.0]
        # self.config['min_z'] = -0.01  # min z to remove plane when extract normals
        # self.config['voxel_grid_dim_x'] = 240
        # self.config['voxel_grid_dim_y'] = 320
        # self.config['voxel_grid_dim_z'] = 280
        # self.config['voxel_grid_origin_x'] = 0.4
        # self.config['voxel_grid_origin_y'] = -0.3
        # self.config['voxel_grid_origin_z'] = -0.2

        # new
        self.config['work_space'] = [0.33, 0.7, -0.20, 0.25, 0.1, 0.5]  # workspace for fusion_py and create one view cloud
        self.config['min_z'] = 0.265  # min z to remove plane when extract normals
        self.config['voxel_grid_dim_x'] = 300
        self.config['voxel_grid_dim_y'] = 500
        self.config['voxel_grid_dim_z'] = 500
        self.config['voxel_grid_origin_x'] = self.config['work_space'][0]
        self.config['voxel_grid_origin_y'] = self.config['work_space'][2]
        self.config['voxel_grid_origin_z'] = self.config['work_space'][4]

        self.config['fusion_script_path'] = os.path.join(utils.get_curr_dir(), 'fusion.py')
        self.config['fusion_exe_path'] = os.path.join(os.path.dirname(utils.get_curr_dir()), 'tsdf_fusion')
        self.config['post_process_exe_path'] = os.path.join(os.path.dirname(utils.get_curr_dir()), 'post_process/build')

        self.topics_to_bag = [
            "/tf",
            "/tf_static",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/aligned_depth_to_color/camera_info",
            "/camera_info",  # "/camera/color/camera_info"，使用自行发布的彩色相机内参，不使用realsense-ros发布的
            "/camera/color/image_raw"
        ]

        self.topics_dict = dict()
        self.topics_dict['rgb'] = "/camera/color/image_raw"
        self.topics_dict['depth'] = "/camera/aligned_depth_to_color/image_raw"
        self.topics_dict['camera_info'] = "/camera_info",  # "/camera/color/camera_info"，使用自行发布的彩色相机内参，不使用realsense-ros发布的

    def setupTF(self):
        tfWrapper = TFWrapper()
        tfBuffer = tfWrapper.getBuffer()
        if self.tfBuffer is None:
            self.tfBuffer = tfBuffer

        self.tf_broadcaster = tf2_ros.TransformBroadcaster()

    def startImageSubscribers(self):
        self.subscribers = {}
        self.subscribers['rgb'] = rospy.Subscriber(self.topics_dict['rgb'], sensor_msgs.msg.Image)
        self.subscribers['depth'] = rospy.Subscriber(self.topics_dict['depth'], sensor_msgs.msg.Image)

    def stopImageSubscribers(self):
        self.subscribers['rgb'].unregister()
        self.subscribers['depth'].unregister()

    def get_bag_folder(self, log_subdir="raw"):
        """ get the folder name to store bag files"""
        base_path = os.path.join(os.path.dirname(utils.get_curr_dir()), self.config['bag_folder'])
        log_id_name = self.config['obj_name'] + '-' + utils.get_current_YYYY_MM_DD_hh_mm_ss()
        bagfile_directory = os.path.join(base_path, log_id_name, log_subdir)
        bagfile_name = "fusion_" + log_id_name + ".bag"

        path_to_bag_file = os.path.join(bagfile_directory, bagfile_name)

        # make bagfile directory with name
        os.system("mkdir -p " + bagfile_directory)

        return path_to_bag_file

    def start_bagging(self, path_to_bag_file, bag_node_name):
        # create parent folder if it doesn't exist
        parent_folder = os.path.dirname(path_to_bag_file)
        if not os.path.exists(parent_folder):
            os.makedirs(parent_folder)

        # start image subscribers
        self.startImageSubscribers()

        # sleep to allow for xtion driver compression issues to be resolved        
        rospy.sleep(self.config['sleep_time_before_bagging'])

        # build up command string
        rosbag_cmd = "rosbag record __name:=" + bag_node_name
        rosbag_cmd += " -O " + path_to_bag_file
        for i in self.topics_to_bag:
            rosbag_cmd += " " + i
        print "[INFO] Rosbag cmd:", rosbag_cmd

        # start bagging
        self.bagging = True
        rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=parent_folder)

        rospy.loginfo("started image subscribers, sleeping for %d seconds", self.config['sleep_time_before_bagging'])

        return path_to_bag_file

    def stop_bagging(self, bag_node_name):
        """
        Stops ROS bagging
        :return:
        :rtype:
        """
        cmd = "rosnode kill /" + bag_node_name
        print "cmd", cmd
        os.system(cmd)

        self.bagging = False
        self.stopImageSubscribers()

    @staticmethod
    def get_numpy_position_from_pose(pose):
        x = pose["camera_to_world"]["translation"]["x"]
        y = pose["camera_to_world"]["translation"]["y"]
        z = pose["camera_to_world"]["translation"]["z"]
        return np.asarray([x, y, z])

    @staticmethod
    def get_quaternion_from_pose(pose):
        quat = utils.getQuaternionFromDict(pose["camera_to_world"])
        x = quat["x"]
        y = quat["y"]
        z = quat["z"]
        w = quat["w"]
        return np.asarray([w, x, y, z])

    def capture_scene(self, use_close_up_poses=False):
        """
        This "moves around and captures all of the data needed for fusion". I.e., it:

        1. Moves the robot "home"
        2. Starts bagging all data needed for fusion
        3. Moves the robot around
        4. Stops bagging data
        5. Moves the robot back home

        This is not a service handler itself, but intended to be modularly called by service handlers.

        :return: bag_filepath, the full path to where (one) of the rosbag (fusion-*.bag) was saved
        :rtype: string

        """
        print "[DEBUG] Start full capture."

        start_pose_name = "test_4"
        # pose_list = ["fusion_left3", "fusion_left1", "fusion_3", "fusion_right1", "fusion_right3"]
        # pose_list = ["fusion_left1", "fusion_left5", "fusion_3", "fusion_right5", "fusion_right1"]
        # pose_list = ["view_right", "middle_fusion", "view_left"]

        # For full table
        pose_list = ["fusion_1", "fusion_1_rot", "fusion_2", "fusion_3_rot", "fusion_3"]
        # For high accuracy
        # pose_list = ["fusion_new_left", "fusion_new_left_rot", "fusion_middle", "fusion_new_right_rot", "fusion_new_right"]

        # Step1: move robot to home
        print "[INFO] Move robot to home"
        self.robot_move.set_vel_scaling(0.5)
        self.robot_move.move_to_pose_named(start_pose_name)
        self.robot_move.move_to_pose_named(pose_list[0])  # FIXME

        # Step2: start bagging for far out data collection
        print "[INFO] Start bagging"
        path_to_bagfile = self.get_bag_folder()
        print "[INFO] Path to bag file: %s" % path_to_bagfile
        self.start_bagging(path_to_bagfile, "bag_node")

        # Step3: moving robot through regular scan poses
        print "[INFO] Moving robot through poses"
        self.robot_move.set_vel_scaling(0.15)

        # move to poses continue
        self.robot_move.move_to_poses_named(pose_list)  # FIXME

        # Step4: stop bagging
        self.stop_bagging("bag_node")

        # wait for bagfile saving
        rospy.sleep(1.0)

        # Optional step: move robot through close up poses, 将舍弃不用
        path_to_close_up_bagfile = os.path.dirname(path_to_bagfile) + "/close_up.bag"
        if use_close_up_poses:  # 为了采集合适的单视角点云
            print "[DEBUG] Start close up capture."
            self.robot_move.set_vel_scaling(0.5)

            pose_list = ["capture_l1", "capture_m1", "capture_r1", "capture_r2", "capture_m2", "capture_l2"]

            # move to first pose before we start bagging
            print "[INFO] Moving robot through close up scan poses"
            self.robot_move.move_to_pose_named(start_pose_name)
            self.robot_move.move_to_pose_named(pose_list[0])

            # start bagging
            print "[INFO] Path to close up poses bag file: %s" % path_to_close_up_bagfile
            self.start_bagging(path_to_close_up_bagfile, "bag_node")

            self.robot_move.set_vel_scaling(0.15)

            # move the robot through close up poses
            for pose in pose_list:
                self.robot_move.move_to_pose_named(pose)
                stamp = [pose, rospy.Time.now().to_nsec()]
                print "[DEBUG] Move to pose:%s done, time_stamp:%d" % (pose, rospy.Time.now().to_nsec())
                self.stamp_ls.append(stamp)
            # self.robot_move.move_to_poses_named(pose_list)

            self.stop_bagging("bag_node")

        # 采集单视角点云
        path_to_one_view_cloud = os.path.dirname(os.path.dirname(path_to_bagfile)) + "/one_view_clouds"
        os.system("mkdir -p " + path_to_one_view_cloud)

        # pose_list = ["capture_l1", "capture_m1", "capture_r1", "capture_r2", "capture_m2", "capture_l2"]
        pose_list = ["one-view1", "one-view2", "one-view3", "one-view4", "one-view5", "one-view6"]

        # move to first pose before we start bagging
        print "[INFO] Moving robot to take one-view clouds"
        self.robot_move.set_vel_scaling(0.8)
        # self.robot_move.move_to_pose_named(start_pose_name)
        # self.robot_move.move_to_pose_named(pose_list[0])

        # move the robot through close up poses
        for i, pose in enumerate(pose_list):
            self.robot_move.move_to_pose_named(pose)
            print "[DEBUG] Move to pose:%s done, take one-view cloud"
            rospy.sleep(0.2)
            get_camera_pose_image(self.im_converter, i, path_to_one_view_cloud, self.config['min_z'])

        # Step5: move back home
        print "[INFO] Move robot back to home"
        self.robot_move.set_vel_scaling(0.4)
        self.robot_move.move_to_pose_named(start_pose_name)

        # wait for bagfile saving
        rospy.sleep(1.0)
        # exit()

        return path_to_bagfile, path_to_close_up_bagfile

    def extract_data_from_rosbag(self, bag_filepath, keep_raw=True, extract_close_up=False, images_dir=None, stamp_ls=None, rgb_only=False):
        """
        This wraps the ImageCapture calls to load and process the raw rosbags, to prepare for fusion.

        :param: bag_filepath, the full path to where the rosbag (fusion-*.bag) was saved
        :ptype: string

        :return: data dir, images_dir the full path to the directory where all the extracted data is saved
                            and its images subdirectory
        :rtype: two strings, separated by commas
        """

        # extract RGB and Depth images from Rosbag
        rgb_topic = self.topics_dict['rgb']
        depth_topic = self.topics_dict['depth']
        camera_info_topic = self.topics_dict['camera_info']

        if images_dir is None:
            log_dir = os.path.dirname(os.path.dirname(bag_filepath))
            processed_dir = os.path.join(log_dir, 'processed')
            images_dir = os.path.join(processed_dir, 'images')
            if extract_close_up:
                images_dir = os.path.join(processed_dir, 'images_close_up')

        print "Using images_dir %s" % images_dir
        image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
                                     self.config['camera_frame'], self.config['world_frame'])
        image_capture.load_ros_bag(bag_filepath)
        image_capture.process_ros_bag(image_capture.ros_bag, images_dir, stamp_ls=stamp_ls, rgb_only=rgb_only)

        if not keep_raw:
            os.remove(bag_filepath)

        rospy.loginfo("Finished writing extracted images to disk")

        return images_dir

    @staticmethod
    def format_data_for_tsdf(image_folder):
        """
        Processes the data into the format needed for tsdf-fusion algorithm
        """

        def cal_camera_matrix(k_matrix):
            matrix = np.zeros((3, 3))
            matrix[0, :3] = k_matrix[0:3]
            matrix[1, :3] = k_matrix[3:6]
            matrix[2, :3] = K_matrix[6:9]
            # print "[INFO] Camera intrinsis matrix:\n", matrix
            return matrix

        def cal_camera_pose(pose_data):
            trans = pose_data['translation']
            quat = pose_data['quaternion']

            trans_xyz = (trans['x'], trans['y'], trans['z'])
            quat_wxyz = (quat['w'], quat['x'], quat['y'], quat['z'])

            # print trans_xyz
            # print quat_wxyz

            # quaternions to rotation matrix
            rotation_matrix = quaternions.quat2mat(quat_wxyz)
            # print rotation_matrix

            # generate homogenous matrix
            matrix = np.zeros((4, 4))
            matrix[:3, :3] = rotation_matrix
            matrix[:3, 3] = np.array(trans_xyz).T
            matrix[3][3] = 1.0
            # print "Camera pose matrix:\n", matrix
            return matrix

        # generate camera matrix file
        camera_info_yaml = os.path.join(image_folder, "camera_info.yaml")
        camera_info = utils.getDictFromYamlFilename(camera_info_yaml)
        K_matrix = camera_info['camera_matrix']['data']
        matrix = cal_camera_matrix(K_matrix)
        camera_info_file_full_path = os.path.join(image_folder, "camera-intrinsics.txt")
        np.savetxt(camera_info_file_full_path, matrix)

        # generate camera pose file
        pose_data_yaml = os.path.join(image_folder, "pose_data.yaml")
        with open(pose_data_yaml, 'r') as stream:
            try:
                pose_data_dict = yaml.load(stream)
            except yaml.YAMLError as exc:
                print(exc)

        for i in pose_data_dict:
            matrix = cal_camera_pose(pose_data_dict[i]['camera_to_world'])

            depth_image_filename = pose_data_dict[i]['depth_image_filename']
            prefix = depth_image_filename.split("depth")[0]
            pose_file_name = prefix + "pose.txt"
            pose_file_full_path = os.path.join(image_folder, pose_file_name)
            np.savetxt(pose_file_full_path, matrix)

        return len(pose_data_dict)

    @staticmethod
    def downsample_by_pose_difference_threshold(images_dir_full_path, linear_distance_threshold=0.03,
                                                rotation_angle_threshold=10, keep_raw=True):
        pose_yaml = os.path.join(images_dir_full_path, "pose_data.yaml")
        pose_dict = utils.getDictFromYamlFilename(pose_yaml)

        images_dir_temp_path = images_dir_full_path + "_downsampled"
        if not os.path.isdir(images_dir_temp_path):
            os.makedirs(images_dir_temp_path)

        previous_pose_pos = FusionServer.get_numpy_position_from_pose(pose_dict[0])
        previous_pose_quat = FusionServer.get_quaternion_from_pose(pose_dict[0])

        num_kept_images = 0
        num_deleted_images = 0

        pose_dict_downsampled = dict()

        img_indices = pose_dict.keys()
        img_indices.sort()
        num_original_images = len(img_indices)

        for i in img_indices:
            single_frame_data = pose_dict[i]
            this_pose_pos = FusionServer.get_numpy_position_from_pose(single_frame_data)
            this_pose_quat = FusionServer.get_quaternion_from_pose(single_frame_data)

            linear_distance = np.linalg.norm(this_pose_pos - previous_pose_pos)

            rotation_distance = utils.compute_angle_between_quaternions(this_pose_quat, previous_pose_quat)

            if i == 0:
                keep_image = True
                num_kept_images += 1
            elif (linear_distance > linear_distance_threshold) or (
                    np.rad2deg(rotation_distance) > rotation_angle_threshold):
                previous_pose_pos = this_pose_pos
                previous_pose_quat = this_pose_quat
                num_kept_images += 1
            else:
                # the pose wasn't sufficiently different
                continue

            # if we have gotten here, then move the images over to the new directory
            pose_dict_downsampled[i] = single_frame_data
            rgb_filename = os.path.join(images_dir_full_path, single_frame_data['rgb_image_filename'])
            rgb_filename_temp = os.path.join(images_dir_temp_path, single_frame_data['rgb_image_filename'])

            shutil.move(rgb_filename, rgb_filename_temp)

            depth_filename = os.path.join(images_dir_full_path, single_frame_data['depth_image_filename'])
            depth_filename_temp = os.path.join(images_dir_temp_path, single_frame_data['depth_image_filename'])
            shutil.move(depth_filename, depth_filename_temp)
            # # delete pose from posegraph
            # del posegraph_list[i-num_deleted_images]
            # num_deleted_images += 1

        # write downsamples pose_data.yaml (forward kinematics)
        utils.saveToYaml(pose_dict_downsampled, os.path.join(images_dir_temp_path, 'pose_data.yaml'))

        # copy camera_info.yaml
        shutil.copy(os.path.join(images_dir_full_path, 'camera_info.yaml'),
                    os.path.join(images_dir_temp_path, 'camera_info.yaml'))

        if not keep_raw:
            # remove old images
            shutil.move(os.path.join(images_dir_full_path, 'camera_info.yaml'),
                        os.path.join(images_dir_temp_path, 'camera_info.yaml'))
            shutil.rmtree(images_dir_full_path)

            # print "renaming %s to %s " % (images_dir_temp_path, images_dir_full_path)

            # rename temp images to images
            os.rename(images_dir_temp_path, images_dir_full_path)

        print "Previously: %d images" % num_original_images
        print "After: %d images" % num_kept_images

        if keep_raw:
            return images_dir_temp_path
        else:
            return images_dir_full_path

    def tsdf_fusion_py(self, images_dir):
        print "\n[INFO] Formatting data for tsdf fusion"
        self.format_data_for_tsdf(images_dir)

        mesh_save_dir = os.path.dirname(images_dir)

        fusion_exe_path = self.config['fusion_script_path']
        fusion_cmd = "python " + fusion_exe_path
        os.system("%s %s %s %s %s %s %s %s %s %s" % (fusion_cmd, images_dir, mesh_save_dir,
                                                     str(self.config['fusion_voxel_size']),
                                                     self.config['work_space'][0], self.config['work_space'][1],
                                                     self.config['work_space'][2], self.config['work_space'][3],
                                                     self.config['work_space'][4], self.config['work_space'][5]))
        print "[INFO] Fusion cmd:", fusion_cmd

    def tsdf_fusion_cpp(self, images_dir):
        start = time.time()

        print "\n[INFO] Formatting data for tsdf fusion"
        self.format_data_for_tsdf(images_dir)

        fusion_exe_path = self.config['fusion_exe_path'] + '/fusion'  # fusion
        convert_script_path = self.config['fusion_exe_path'] + '/tsdf_bin_to_ply.py'
        extract_exe_path = self.config['post_process_exe_path'] + '/extract_normals'

        mesh_save_dir = os.path.dirname(images_dir)

        # run tsdf fusion
        # fusion_cmd: exe_path images_dir save_dir voxel_size dim_x dim_y dim_z origin_x origin_y origin_z
        fusion_cmd = "%s %s %s %s %s %s %s %s %s %s" % \
                     (fusion_exe_path, images_dir, mesh_save_dir, str(self.config['fusion_voxel_size']),
                      str(self.config['voxel_grid_dim_x']), str(self.config['voxel_grid_dim_y']),
                      str(self.config['voxel_grid_dim_z']), str(self.config['voxel_grid_origin_x']),
                      str(self.config['voxel_grid_origin_y']), str(self.config['voxel_grid_origin_z']))
        print "[INFO] Fusion cmd:", fusion_cmd
        os.system(fusion_cmd)

        # convert tsdf.bin to mesh and cumpute normals
        # covert_cmd: script_path input_dir output_dir
        covert_cmd = "python %s %s %s" % (convert_script_path, mesh_save_dir, mesh_save_dir)
        print "[INFO] Covert cmd:", covert_cmd
        os.system(covert_cmd)

        # extract surface normals from mesh
        # extract_cmd: exe_path input_dir output_dir
        extract_cmd = "%s %s %s %s" % (extract_exe_path, mesh_save_dir, mesh_save_dir, self.config['min_z'])
        print "[INFO] Extract cmd:", extract_cmd
        os.system(extract_cmd)

        print "[INFO] Tsdf fusion and post processing took: %.3fs" % (time.time()-start)

    def create_point_cloud(self, images_dir, save_dir=''):
        print "[INFO] Create point cloud for images dir:", images_dir
        creat_cloud_exe_path = self.config['post_process_exe_path'] + '/create_point_cloud'

        pcd_save_dir = os.path.join(os.path.dirname(images_dir), save_dir)
        if not os.path.exists(pcd_save_dir):
            os.mkdir(pcd_save_dir)

        # create point cloud from rgb and depth image
        def worker(creat_cloud_cmd):
            os.system(creat_cloud_cmd)

        imagels = sorted(glob.glob(os.path.join(images_dir, '*depth.png')))
        print imagels

        work_num = 0
        for image in imagels:
            dirname = os.path.dirname(image)
            basename = os.path.basename(image)
            suffix = basename.split("_")[-1]
            prefix = basename[:-len(suffix)-1]

            # color_img = os.path.join(dirname, prefix + '_rgb.png')
            color_img = os.path.join(dirname, prefix + '_rgb.jpg')
            depth_img = os.path.join(dirname, prefix + '_depth.png')
            camera_pose = os.path.join(dirname, prefix + '_pose.txt')
            cloud_pcd = os.path.join(pcd_save_dir, prefix + '_cloud.pcd')

            # creat_cloud_cmd: exe_path color_path depth_path cam_K_dir camera_pose save_path -x x -y y -z z
            creat_cloud_cmd = "%s %s %s %s %s %s %s %s %s %s %s %s" % \
                              (creat_cloud_exe_path, color_img, depth_img, images_dir, camera_pose, cloud_pcd,
                               self.config['work_space'][0], self.config['work_space'][1],
                               self.config['work_space'][2], self.config['work_space'][3],
                               self.config['work_space'][4], self.config['work_space'][5])

            print "Creat cloud:", cloud_pcd
            # print "creat_cloud_cmd", creat_cloud_cmd
            # os.system(creat_cloud_cmd)

            if not os.path.exists(camera_pose):
                print "[DEBUG] Format data for create point cloud."
                self.format_data_for_tsdf(images_dir)

            if os.path.exists(color_img) and os.path.exists(camera_pose):  # check
                p = multiprocessing.Process(target=worker, args=(creat_cloud_cmd,))
                p.start()
                work_num += 1
            else:
                print "[WARN] No image or pose data when create: " + cloud_pcd

        while True:  # wait for all work done
            cloudls = glob.glob(os.path.join(pcd_save_dir, '*pcd'))
            if work_num == len(cloudls):
                print "[INFO] Point clouds saved to:", pcd_save_dir
                break

    def handle_capture_scene_and_fuse(self, bag_filepath=None, close_up_bag_filepath=None):
        """
        NOTE: The coordinate of tsdf is camera's world frame, we describe camera's pose use base_link->camera
        so the tsdf's coordinate is robot's "base_link"
        """
        start = time.time()
        print "\n[INFO] Handling capture_scene_and_fuse"

        print "\n[INFO] Start capture scene"
        if bag_filepath is None:
            bag_filepath, close_up_bag_filepath = self.capture_scene(self.config['use_close_up_poses'])

        print "\n[INFO] Extract images from bag"
        images_dir = self.extract_data_from_rosbag(bag_filepath, keep_raw=self.config['keep_raw_bag'])
        data_dir = os.path.dirname(os.path.dirname(images_dir))

        print "\n[INFO] Running tsdf fusion"
        downsampled_images_dir = self.downsample_by_pose_difference_threshold(images_dir, 0.01, 5, keep_raw=self.config['keep_raw_images'])
        self.tsdf_fusion_cpp(downsampled_images_dir)
        self.tsdf_fusion_py(downsampled_images_dir)

        if self.config['use_close_up_poses']:
            print "\n[INFO] Extract close_up images from bag"
            close_up_images_dir = self.extract_data_from_rosbag(close_up_bag_filepath, keep_raw=self.config['keep_raw_bag'],
                                                                extract_close_up=True, stamp_ls=self.stamp_ls)

            print "\n[INFO] Create point cloud"
            close_up_downsampled_images_dir = self.downsample_by_pose_difference_threshold(close_up_images_dir, 0.01, 5, keep_raw=self.config['keep_raw_images'])
            # self.create_point_cloud(close_up_downsampled_images_dir, save_dir="clouds")

            stamped_img_output_dir = os.path.join(os.path.dirname(close_up_images_dir), "images_stamped")
            self.create_point_cloud(stamped_img_output_dir, save_dir="images_stamped")

        print "\n[INFO] Handle capture scene and fuse finished!"
        print "\n[INFO] Data saved to dir: %s" % data_dir

        # check the result of fusion
        plyls = glob.glob(os.path.join(data_dir, 'processed', '*.ply'))
        if len(plyls) == 0:
            raise Exception("Tsdf fusion failed, please check the problem!")

        print "\n[INFO] Fusion took: %ds" % (time.time()-start)


if __name__ == "__main__":
    rospy.init_node("fusion_server")
    fs = FusionServer()

    # ###############    test bagging    ###############
    # fs.start_bagging()
    # rospy.sleep(2)
    # fs.stop_bagging()

    # #############    test extract data    ############
    # close_up_bag_filepath = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-20-06-02/raw/close_up.bag"
    # close_up_images_dir = fs.extract_data_from_rosbag(close_up_bag_filepath, keep_raw=True, extract_close_up=True)
    # exit()

    # bag_filepath = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2019-10-27-15-57-08/raw/fusion_2019-10-27-15-57-08.bag"
    # images_dir = fs.extract_data_from_rosbag(bag_filepath, rgb_only=False)

    # ############    test format data    ##############
    # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-01-21-35-13/processed/images"
    # data_cnt = fs.format_data_for_tsdf(images_dir)

    # #############    test downsample    ###############
    # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-16-39-45/processed/images_close_up"
    # fs.downsample_by_pose_difference_threshold(images_dir)
    # exit()

    # ###########    test extrac normals    #############
    # save_dir = os.path.dirname(images_dir)
    # extract_exe_path = fs.config['post_process_exe_path'] + '/extract_normals'
    # extract_cmd = "%s %s %s %s" % (extract_exe_path, save_dir, save_dir, fs.config['min_z'])
    # print "[INFO] Extract cmd:", extract_cmd
    # os.system(extract_cmd)

    # #############    test tsdf fusion    ##############
    # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-20-06-02/processed/images"
    # # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-20-06-02/processed/images_close_up"
    # fs.tsdf_fusion_py(images_dir)
    # exit()

    # fs.tsdf_fusion_cpp(images_dir)
    # print(images_dir)
    # exit()

    # #############    test cloud create    ##############
    # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-15-15-46-44/processed/images"
    # fs.config['work_space'] = [-4.0, 4.0, -4.0, 4.0, -4.0, 4.0]
    # fs.create_point_cloud(images_dir, save_dir="clouds")
    # exit()

    # ###########    test reload bag file    ############
    # bag_filepath = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-20-06-02/raw/fusion_2020-01-13-20-06-02.bag"
    # close_up_bag_filepath = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/2020-01-13-20-06-02/raw/close_up.bag"
    # fs.handle_capture_scene_and_fuse(bag_filepath, close_up_bag_filepath)
    # exit()

    # ###########    test get camera pose image    ############
    # path = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/data/multi-view"
    # im_converter = image_converter()
    # get_camera_pose_image(im_converter, 0, path)

    # #################    test all    ##################
    fs.handle_capture_scene_and_fuse()
