#!/usr/bin/env python

# system
import os
import time
import subprocess
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

ROS_BAGGING_NODE_NAME = "rosbag_node"


class RobotMove(object):
    def __init__(self):
        pass

    @staticmethod
    def move_to_pose_named(pose_name):
        rospy.wait_for_service('move_to_pose_named')
        try:
            move_to_pose_named = rospy.ServiceProxy('move_to_pose_named', MoveToPoseNamed)
            resp = move_to_pose_named(pose_name)
            if resp.success:
                print "[INFO] Robot move to pose named %s success." % pose_name
                return True
        except rospy.ServiceException, e:
            print "Robot move service call failed: %s" % e


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
                except tf2_ros.LookupException, e:
                    print "[Exception] tf2_ros.LookupException:\n", e
                    print "wasn't able to get transform for image message %d, skipping" % counter
                    continue

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

            rgb_filename = "frame-%06i.%s.jpg" % (idx, "color")
            rgb_filename_full = os.path.join(output_dir, rgb_filename)

            depth_filename = "frame-%06i.%s.png" % (idx, "depth")
            depth_filename_full = os.path.join(output_dir, depth_filename)

            if idx % log_rate == 0:
                print "writing image %d to file %s" % (idx, rgb_filename)

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

        # NOTE: currently the batch_extract_and_fuse_all_scenes.py
        # script checks for the existence of this file (camera_info.yaml) 
        # to determine if the extraction process was completed.

        utils.saveToYaml(camera_info_dict, os.path.join(output_dir, 'camera_info.yaml'))

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

    def setupConfig(self):
        self.config = dict()
        self.config['world_frame'] = "base_link"
        self.config['camera_frame'] = "camera_color_optical_frame"
        self.config['sleep_time_before_bagging'] = 3.0
        self.config['bag_folder'] = "data"
        self.config['use_close_up_poses'] = False

        self.config['fusion_voxel_size'] = 0.004
        self.config['fusion_max_depth'] = 1.2
        self.config['fusion_exe_path'] = os.path.join(utils.get_curr_dir(), 'fusion.py')

        self.topics_to_bag = [
            "/tf",
            "/tf_static",
            "/camera/aligned_depth_to_color/image_raw",
            "/camera/aligned_depth_to_color/camera_info",
            "/camera/color/camera_info",
            "/camera/color/image_raw"
        ]

        self.topics_dict = dict()
        self.topics_dict['rgb'] = "/camera/color/image_raw"
        self.topics_dict['depth'] = "/camera/aligned_depth_to_color/image_raw"
        self.topics_dict['camera_info'] = "/camera/color/camera_info"

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
        log_id_name = utils.get_current_YYYY_MM_DD_hh_mm_ss()
        bagfile_directory = os.path.join(base_path, log_id_name, log_subdir)
        bagfile_name = "fusion_" + log_id_name + ".bag"

        path_to_bag_file = os.path.join(bagfile_directory, bagfile_name)

        # make bagfile directory with name
        os.system("mkdir -p " + bagfile_directory)

        return path_to_bag_file

    def start_bagging(self, path_to_bag_file):
        # create parent folder if it doesn't exist
        parent_folder = os.path.dirname(path_to_bag_file)
        if not os.path.exists(parent_folder):
            os.makedirs(parent_folder)

        # start image subscribers
        self.startImageSubscribers()

        # sleep to allow for xtion driver compression issues to be resolved        
        rospy.sleep(self.config['sleep_time_before_bagging'])

        # build up command string
        rosbag_cmd = "rosbag record __name:=" + ROS_BAGGING_NODE_NAME
        rosbag_cmd += " -O " + path_to_bag_file
        for i in self.topics_to_bag:
            rosbag_cmd += " " + i
        print "[INFO] Rosbag cmd:", rosbag_cmd

        # start bagging
        self.bagging = True
        rosbag_proc = subprocess.Popen(rosbag_cmd, stdin=subprocess.PIPE, shell=True, cwd=parent_folder)

        rospy.loginfo("started image subscribers, sleeping for %d seconds", self.config['sleep_time_before_bagging'])

        return path_to_bag_file

    def stop_bagging(self):
        """
        Stops ROS bagging
        :return:
        :rtype:
        """
        cmd = "rosnode kill /" + ROS_BAGGING_NODE_NAME
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

        # Step1: move robot to home
        print "[INFO] Move robot to home"
        self.robot_move.move_to_pose_named("home")

        # Step2: start bagging for far out data collection
        print "[INFO] Start bagging"
        path_to_bagfile = self.get_bag_folder()
        print "[INFO] Path to bag file: %s" % path_to_bagfile
        self.start_bagging(path_to_bagfile)

        # Step3: moving robot through regular scan poses
        print "[INFO] Moving robot through poses"
        # pose_list = ["test_1", "test_2", "test_3", "test_4", "pick_1", "pick_2", "pick_3", "pick_4"]
        pose_list = ["pick_1", "pick_2", "pick_3", "pick_4"]
        for pose in pose_list:
            self.robot_move.move_to_pose_named(pose)

        # Step4: stop bagging
        self.stop_bagging()

        # Optional step: move robot through close up poses
        if use_close_up_poses:
            # move to first pose before we start bagging
            print "[INFO] Moving robot through close up scan poses"
            self.robot_move.move_to_pose_named("home")

            # start bagging
            path_to_bagfile = self.get_bag_folder("raw_close_up")
            print "[INFO] Path to close up poses bag file: %s" % path_to_bagfile
            self.start_bagging(path_to_bagfile)

            # move the robot through close up poses
            pose_list = ["test_1", "test_2", "test_3", "test_4"]
            for pose in pose_list:
                self.robot_move.move_to_pose_named(pose)

            self.stop_bagging()

        # Step5: move back home
        print "[INFO] Move robot back to home"
        self.robot_move.move_to_pose_named("home")

        # wait for bagfile saving
        rospy.sleep(1.0)
        return path_to_bagfile

    def extract_data_from_rosbag(self, bag_filepath, images_dir=None, rgb_only=False):
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

        print "Using images_dir %s" % images_dir
        image_capture = ImageCapture(rgb_topic, depth_topic, camera_info_topic,
                                     self.config['camera_frame'], self.config['world_frame'])
        image_capture.load_ros_bag(bag_filepath)
        image_capture.process_ros_bag(image_capture.ros_bag, images_dir, rgb_only=rgb_only)

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

        images_dir_temp_path = os.path.join(os.path.dirname(images_dir_full_path), 'images_downsampled')
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

    def handle_capture_scene_and_fuse(self):
        """
        NOTE: The coordinate of tsdf is camera's world frame, we describe camera's pose use base_link->camera
        so the tsdf's coordinate is robot's "base_link"
        """

        print "\n[INFO] Handling capture_scene_and_fuse"

        print "\n[INFO] Start capture scene"
        bag_filepath = self.capture_scene(self.config['use_close_up_poses'])

        print "\n[INFO] Extract images from bag"
        images_dir = self.extract_data_from_rosbag(bag_filepath)
        data_dir = os.path.dirname(os.path.dirname(images_dir))

        print "\n[INFO] Formatting data for tsdf fusion"
        self.format_data_for_tsdf(images_dir)

        print "\n[INFO] Running tsdf fusion"
        fusion_exe_path = self.config['fusion_exe_path']
        fusion_cmd = "python " + fusion_exe_path
        os.system("%s %s %s %s" % (fusion_cmd, images_dir, str(self.config['fusion_voxel_size']),
                                   str(self.config['fusion_max_depth'])))
        print "[INFO] Fusion cmd:", fusion_cmd

        print "\n[INFO] Downsampling image folder"
        self.downsample_by_pose_difference_threshold(images_dir, 0.03, 10)

        print "\n[INFO] Handle capture scene and fuse finished!"
        print "\n[INFO] Data saved to dir: %s" % data_dir

        # check the result of fusion
        plyls = glob.glob(os.path.join(data_dir, 'processed', '*.ply'))
        if len(plyls) == 0:
            raise Exception("Tsdf fusion failed, please check the problem!")


if __name__ == "__main__":
    rospy.init_node("fusion_server")
    fs = FusionServer()

    # ###############    test bagging    ###############
    # fs.start_bagging()
    # rospy.sleep(2)
    # fs.stop_bagging()

    # #############    test extract data    ############
    # bag_filepath = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/reference/data/2019-10-24-20-09-02/raw/fusion_2019-10-24-20-09-02.bag"
    # images_dir = fs.extract_data_from_rosbag(bag_filepath, rgb_only=False)

    # ############    test format data    ##############
    # images_dir = "/home/sdhm/catkin_ws/src/mantra_robot/mantra_application/reconstruction/reference/data/2019-10-25-22-03-45/processed/images"
    # data_cnt = fs.format_data_for_tsdf(images_dir)

    # #############    test tsdf fusion    ##############
    # fusion_exe_path = os.path.join(os.path.dirname(utils.get_curr_dir()), 'scripts', 'fusion.py')
    # command = "python " + fusion_exe_path
    # os.system("%s %s" % (command, images_dir))

    # #############    test downsample    ###############
    # fs.downsample_by_pose_difference_threshold(images_dir)

    # #################    test all    ##################
    fs.handle_capture_scene_and_fuse()
