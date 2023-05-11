#!/usr/bin/env python3
import rosbag
import cv2
import os
import numpy as np
from cv_bridge import CvBridge
import argparse


def main(topics_to_extract, bag_path):
    # Set bag file path
    # "/home/chun/Downloads/20230216_130118_archive/20230216_130118_archive_extracted.bag"
    # "./*.bag"
    bag_directory = bag_path[0:bag_path.rindex('/')]

    output_paths = {}

    # Set output directory path
    # Create output directory if it doesn't exist
    for topic_ in topics_to_extract:
        output_paths[topic_] = os.path.join(bag_directory, topic_.replace('/', '-'))
        if not os.path.exists(output_paths[topic_]):
            os.makedirs(output_paths[topic_])

    # Open bag file
    bag = rosbag.Bag(bag_path)
    bridge = CvBridge()
    # Loop through bag file and extract images
    for topic, msg, t in bag.read_messages(topics = topics_to_extract):
        # Extract image from message
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Construct filename for image
        filename = "image_{}.jpg".format(t.to_nsec())
        cv2.imwrite(os.path.join(output_paths[topic], filename), cv_img)

        # Save image to output directory

    # Close bag file
    bag.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description='extract odom and measure distance')
    parser.add_argument('-b', '--bag_path', type=str, nargs=1, required=True,
                        help='path to the config file')
    parser.add_argument('-t', '--topics', nargs='+', default=[])
    args = parser.parse_args()
    main(args.topics, args.bag_path)
