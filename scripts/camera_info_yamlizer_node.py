#!/usr/bin/env python

import os
import sys
import numpy as np

import rospy
import tf2_ros
import tf.transformations as transformations
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo

class CameraParams:
    def __init__(self, id):
        self.camera_id = id
        self.distortion_model = None
        self.distortion_coeffs = None
        self.intrinsics = None
        self.frame_rate = None
        self.resolution = None
        self.camera_model = None
        self.extrinsics_trans = None
        self.extrinsics_quat = None

    def dump_to_yaml(self, filepath):
        """ Dumps contents of an instance of CameraParams to a yaml file.

            Args:
                filepath: A string representing the absolute path to the output yaml file.
        """
        output = {}
        output["camera_id"] = self.camera_id

        body_pose_cam = transformations.quaternion_matrix(self.extrinsics_quat)
        body_pose_cam[:,3] = np.array([self.extrinsics_trans + [1]])
        body_pose_cam = np.reshape(body_pose_cam, -1).tolist()
        output["T_BS"] = {"cols": 4, "rows": 4, "data": (body_pose_cam)}

        output["rate_hz"] = self.frame_rate
        output["resolution"] = self.resolution
        output["camera_model"] = self.camera_model
        output["intrinsics"] = self.intrinsics
        output["distortion_model"] = self.distortion_model
        output["distortion_coefficients"] = self.distortion_coeffs

        try:
            import ruamel.yaml as yaml
        except ImportError as e:
            raise Exception("ERROR: Did you do `pip install ruamel.yaml`?", e)

        with open(filepath, 'w') as yaml_file:
                yaml.dump(output, yaml_file, default_flow_style=None, version=(1,0))


class CameraInfoYamlizerNode:
    def __init__(self):
        self.left_cam_info_topic = rospy.get_param("~left_cam_info_topic")
        self.right_cam_info_topic = rospy.get_param("~right_cam_info_topic")
        self.output_dir = rospy.get_param("~output_dir")
        self.left_cam_frame_id = rospy.get_param("~left_cam_frame_id")
        self.right_cam_frame_id = rospy.get_param("~right_cam_frame_id")
        self.base_link_frame_id = rospy.get_param("~base_link_frame_id")

        self.left_yaml_file = os.path.join(self.output_dir, "LeftCameraParams.yaml")
        self.right_yaml_file = os.path.join(self.output_dir, "RightCameraParams.yaml")

        self.left_info_sub = rospy.Subscriber(self.left_cam_info_topic,
                                              CameraInfo,
                                              self.left_info_cb,
                                              queue_size=10)
        self.right_info_sub = rospy.Subscriber(self.right_cam_info_topic,
                                               CameraInfo,
                                               self.right_info_cb,
                                               queue_size=10)

        self.left_camera_params = CameraParams("left_cam")
        self.right_camera_params = CameraParams("right_cam")

        self.num_packets_acquired = 0

        self.get_cam_tf()
        self.spin()

    def spin(self):
        """ Main loop for the yamlizer node. Node shutdown occurs once all
            data is collected.
        """
        while self.num_packets_acquired < 4:
            continue
        
        if self.num_packets_acquired == 4:
            self.dump_to_yaml()
            rospy.signal_shutdown("Acquired all data. Dumping to yaml and exiting.")

    def left_info_cb(self, msg):
        """ ROS callback for left camera info messages. Info is converted to a CameraParams
            instance, and the subscriber is shutdown as this is a one-shot process.

            Args:
                msg: A sensor_msgs.msg.CameraInfo object.
        """
        print("Recieved left camera info message")
        self.left_camera_params.distortion_model = msg.distortion_model
        self.left_camera_params.distortion_coeffs = list(msg.D)
        self.left_camera_params.intrinsics = [msg.K[i] for i in [0, 4, 2, 5]]
        self.left_camera_params.frame_rate = 0.0
        self.left_camera_params.resolution = [msg.width, msg.height]
        self.left_camera_params.camera_model = "pinhole"

        self.num_packets_acquired += 1
        self.left_info_sub.unregister()

    def right_info_cb(self, msg):
        """ ROS callback for right camera info messages. Info is converted to a CameraParams
            instance, and the subscriber is shutdown as this is a one-shot process.

            Args:
                msg: A sensor_msgs.msg.CameraInfo object.
        """
        print("Recieved right camera info message")
        self.right_camera_params.distortion_model = msg.distortion_model
        self.right_camera_params.distortion_coeffs = list(msg.D)
        self.right_camera_params.intrinsics = [msg.K[i] for i in [0, 4, 2, 5]]
        self.right_camera_params.frame_rate = 0.0
        self.right_camera_params.resolution = [msg.width, msg.height]
        self.right_camera_params.camera_model = "pinhole"

        self.num_packets_acquired += 1
        self.right_info_sub.unregister()

    def get_cam_tf(self):
        """ Retrieve camera transforms from TF tree for both left and right cameras, and
            updates the CameraParams objects.
        """
        tf_buffer = tf2_ros.Buffer()
        tf_listener = tf2_ros.TransformListener(tf_buffer)

        left_tf = tf_buffer.lookup_transform(self.base_link_frame_id,
                                             self.left_cam_frame_id,
                                             rospy.Time.now(),
                                             rospy.Duration(5.0))

        self.left_camera_params.extrinsics_trans = [left_tf.transform.translation.x,
                                                    left_tf.transform.translation.y,
                                                    left_tf.transform.translation.z]
        self.left_camera_params.extrinsics_quat = [left_tf.transform.rotation.x,
                                                   left_tf.transform.rotation.y,
                                                   left_tf.transform.rotation.z,
                                                   left_tf.transform.rotation.w]
        print("Recieved left camera transform message")
        self.num_packets_acquired += 1

        right_tf = tf_buffer.lookup_transform(self.base_link_frame_id,
                                              self.right_cam_frame_id,
                                              rospy.Time.now(),
                                              rospy.Duration(5.0))

        self.right_camera_params.extrinsics_trans = [right_tf.transform.translation.x,
                                                     right_tf.transform.translation.y,
                                                     right_tf.transform.translation.z]
        self.right_camera_params.extrinsics_quat = [right_tf.transform.rotation.x,
                                                    right_tf.transform.rotation.y,
                                                    right_tf.transform.rotation.z,
                                                    right_tf.transform.rotation.w]
        print("Recieved right camera transform message")
        self.num_packets_acquired += 1

    def dump_to_yaml(self):
        """ Save both left and right camera parameter files to disk.
        """
        self.left_camera_params.dump_to_yaml(self.left_yaml_file)
        self.right_camera_params.dump_to_yaml(self.right_yaml_file)

if __name__ == "__main__":
    rospy.init_node("cam_info_yamlizer_node")
    ciyn = CameraInfoYamlizerNode()
    while not rospy.is_shutdown():
        rospy.spin()
