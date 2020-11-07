#!/usr/bin/env python

import os

import rosbag
import time
from cv_bridge import CvBridge
import tf2_msgs.msg
import pandas as pd
import geometry_msgs.msg
import rospy
import yaml

import subprocess

import shutil as sh

def status(length, percent):
    sys.stdout.write('\x1B[2K') # Erase entire current line
    sys.stdout.write('\x1B[0E') # Move to the beginning of the current line
    progress = "Progress: ["
    for i in range(0, length):
        if i < length * percent:
            progress += '='
        else:
            progress += ' '
    progress += "] " + str(round(percent * 100.0, 2)) + "%"
    sys.stdout.write(progress)
    sys.stdout.flush()

def transform_msg_from_csv(path_to_csv, child_frame_id, frame_id):
    tfs = pd.read_csv(path_to_csv)
    tf_array = []
    for index, tf in tfs.iterrows():
        tf_msg = tf2_msgs.msg.TFMessage()
        tf_stamped = geometry_msgs.msg.TransformStamped()
        tf_stamped.header.frame_id = frame_id
        tf_stamped.child_frame_id = child_frame_id
        # tf_stamped.header.stamp = rospy.Time.from_sec(tf["#timestamp"]*1e-9) # ns to sec
        # Assumes timestamps are in the first column
        tf_stamped.header.stamp = rospy.Time.from_sec(tf[0]*1e-9) # ns to sec
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


def get_rosbag_info(rosbag_path):
    return yaml.load(subprocess.Popen(['rosbag', 'info', '--yaml', rosbag_path], stdout=subprocess.PIPE).communicate()[0], Loader=yaml.FullLoader)


def add_tf_to_rosbag(input_rosbag_path, csv_path, output_rosbag_path, body_frame_id, world_frame_id = 'world', tf_topic = '/tf', tf_array_idx = 0):
    """
    """
    tf_array = transform_msg_from_csv(csv_path, body_frame_id, world_frame_id)

    clean_original_rosbag = False
    original_rosbag_path = ""
    if input_rosbag_path == output_rosbag_path:
        # The user wants to overwrite rosbag
        clean_original_rosbag = True
        original_rosbag_path = os.path.splitext(input_rosbag_path)[0] + ".orig.bag"
        # We copy it since opening the rosbag in write/read mode
        # raises an Unindexed Rosbag Exception.
        sh.copy(input_rosbag_path, original_rosbag_path)
    else:
        original_rosbag_path = input_rosbag_path
        # We copy it since we only add new messages
        sh.copy(input_rosbag_path, output_rosbag_path)

    # This is for logging progress
    info_dict = get_rosbag_info(original_rosbag_path)
    duration = info_dict['duration']
    start_time = info_dict['start']

    # We write the pose msgs in order wrt its timestamp
    try:
        assert(tf_array_idx >= 0)
        assert(tf_array_idx < len(tf_array))
        # USE APPEND mode, since we copy-pasted it before!
        with rosbag.Bag(output_rosbag_path, 'a') as outbag:
            last_time = time.clock() # This is just to log progress
            inbag = rosbag.Bag(original_rosbag_path, 'r')
            for topic, msg, msg_timestamp in inbag.read_messages():
                while(tf_array_idx < len(tf_array) and
                      tf_array[tf_array_idx].transforms[0].header.stamp <= msg_timestamp):
                    pose_msg = tf_array[tf_array_idx]
                    pose_timestamp = pose_msg.transforms[0].header.stamp

                    # Write our tf messages as long as they have an earlier timestamp
                    # than the timestamp of the current rosbag msg
                    outbag.write(tf_topic, pose_msg, pose_timestamp)

                    # Get next pose message
                    tf_array_idx = tf_array_idx + 1

                if time.clock() - last_time > .1:
                    # Log current progress
                    percent = (msg_timestamp.to_sec() - start_time) / duration
                    status(40, percent)
                    last_time = time.clock()


                # Re-write the already present messages
                # TODO(Toni): is this really necessary?
                # outbag.write(topic, msg, msg_timestamp)
                # print "Wrote message with t: {}".format(t)
            status(40, 1)

            # Write the left-over pose messages
            while(tf_array_idx < len(tf_array)):
                pose_msg = tf_array[tf_array_idx]
                pose_timestamp = pose_msg.transforms[0].header.stamp

                outbag.write(tf_topic, pose_msg, pose_timestamp)

                # Go to next pose message
                tf_array_idx = tf_array_idx + 1
    except IOError as error:
        print("Error loading input rosbag: %s" % original_rosbag_path)
        print("Or Error loading output rosbag: %s" % output_rosbag_path)
        print(error)

    if clean_original_rosbag:
        os.remove(original_rosbag_path)

def run(args):
    if not args.output_rosbag_path:
        print("No output rosbag path provided, writing tfs directly on input rosbag.")
        args.output_rosbag_path = args.input_rosbag_path

    add_tf_to_rosbag(args.input_rosbag_path,
                     args.csv_path,
                     args.output_rosbag_path,
                     args.base_link_frame_id,
                     "world", "/tf", 0)
    return True

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

    output_opts.add_argument("--output_rosbag_path", type=str,
                             help="Path to the output rosbag.",
                             default="")

    output_opts.add_argument("--base_link_frame_id", type=str,
                             help="Frame id corresponding to the trajectory csv (i.e. base_link)",
                             default="base_link_dvio")

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
