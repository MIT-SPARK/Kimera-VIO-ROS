#!/usr/bin/env python

import rosbag
import re
from cv_bridge import CvBridge
import cv2
import numpy as np
import pandas as pd
import tf2_msgs.msg
import geometry_msgs.msg
import rospy
from nav_msgs.msg import Odometry

def transform_msg_from_csv(path_to_csv, child_frame_id, frame_id):
    tfs = pd.read_csv(path_to_csv)
    tf_array = []
    for index, tf in tfs.iterrows():
        tf_msg = tf2_msgs.msg.TFMessage()
        tf_stamped = geometry_msgs.msg.TransformStamped()
        tf_stamped.header.frame_id = frame_id
        tf_stamped.child_frame_id = child_frame_id
        tf_stamped.header.stamp = rospy.Time(tf['#timestamp']*1e-9) # ns to sec
        tf_stamped.transform.translation.x = tf['x']
        tf_stamped.transform.translation.y = tf['y']
        tf_stamped.transform.translation.z = tf['z']
        tf_stamped.transform.rotation.x = tf['qx']
        tf_stamped.transform.rotation.y = tf['qy']
        tf_stamped.transform.rotation.z = tf['qz']
        tf_stamped.transform.rotation.w = tf['qw']
        tf_msg.transforms.append(tf_stamped)
        tf_array.append(tf_msg)
    return tf_array


def add_tf_to_rosbag(input_rosbag_path, csv_path, output_rosbag_path, body_frame_id, world_frame_id = 'world', tf_topic = '/tf', tf_array_idx = 0):
    """
    """
    tf_array = transform_msg_from_csv(csv_path, body_frame_id, world_frame_id)
    pose_msg = tf_array[tf_array_idx]
    pose_timestamp = pose_msg.transforms[0].header.stamp


    # We write the pose msgs in order wrt its timestamp
    try:
        with rosbag.Bag(input_rosbag_path, 'w') as outbag:
            for topic, msg, msg_timestamp in rosbag.Bag(output_rosbag_path, 'r').read_messages():
                while(tf_array_idx < len(tf_array) and pose_timestamp <= msg_timestamp):
                    # Write our tf messages
                    outbag.write(tf_topic, pose_msg, pose_timestamp)
                    # print "Interjected IMU message with t: {}".format(tf_array[tf_array_idx].header.stamp)

                    # Go to next pose message
                    tf_array_idx = tf_array_idx + 1
                    pose_msg = tf_array[tf_array_idx]
                    pose_timestamp = pose_msg.transforms[0].header.stamp

                # Re-write the already present messages
                # TODO(Toni): is this really necessary?
                outbag.write(topic, msg, msg_timestamp)
                # print "Wrote message with t: {}".format(t)

            # Write the left-over pose messages
            while(tf_array_idx < len(tf_array)):
                outbag.write(tf_topic, pose_msg, pose_timestamp)

                # Go to next pose message
                tf_array_idx = tf_array_idx + 1
                pose_msg = tf_array[tf_array_idx]
                pose_timestamp = pose_msg.transforms[0].header.stamp
    except IOError as error:
        print("Error loading input rosbag: %s" % input_rosbag_path)
        print("Or Error loading output rosbag: %s" % output_rosbag_path)
        print(error)

def run(args):
    in_place_write = True
    if in_place_write:
        add_tf_to_rosbag(args.input_rosbag_path, args.csv_path, args.input_rosbag_path, "left_cam_dvio", "world", "/tf", 0)
    else:
        add_tf_to_rosbag(args.input_rosbag_path, args.csv_path, args.output_rosbag_path, "left_cam_dvio", "world", "/tf", 0)

def parser():
    import argparse
    basic_desc = "Add poses in a csv file to a rosbag in TF format and time synchronized order."

    shared_parser = argparse.ArgumentParser(add_help=True, description="{}".format(basic_desc))

    input_opts = shared_parser.add_argument_group("input options")
    output_opts = shared_parser.add_argument_group("output options")

    input_opts.add_argument("input_rosbag_path", type=str,
                            help="Path to the input rosbag.",
                            default="./rosbag.bag")
    input_opts.add_argument("csv_path", type=str,
                            help="Path to the csv file with pose information.",
                            default="./traj_vio.csv")

    output_opts.add_argument("output_rosbag_path", type=str,
                             help="Path to the output rosbag.", default="./output.bag")

    main_parser = argparse.ArgumentParser(description="{}".format(basic_desc))
    sub_parsers = main_parser.add_subparsers(dest="subcommand")
    sub_parsers.required = True
    return shared_parser

import argcomplete
import sys
if __name__ == '__main__':
    parser = parser()
    argcomplete.autocomplete(parser)
    args = parser.parse_args()
    # try:
    if run(args):
        sys.exit(os.EX_OK)
    else:
        print("ERROR")
