#!/usr/bin/env python3
import rosbag
import cv2
import os
import numpy as np
from cv_bridge import CvBridge


# Set bag file path
bag_file = "/home/chun/Downloads/20230216_130118_archive/20230216_130118_archive_extracted.bag"

# Set output directory path
r_output_dir = "/home/chun/Downloads/20230216_130118_archive/R"
l_output_dir = "/home/chun/Downloads/20230216_130118_archive/L"

rl_output_dir = "/home/chun/Downloads/20230216_130118_archive/RL"

r_raw_output_dir = "/home/chun/Downloads/20230216_130118_archive/Rraw"
l_raw_output_dir = "/home/chun/Downloads/20230216_130118_archive/Lraw"

# Create output directory if it doesn't exist
if not os.path.exists(r_output_dir):
    os.makedirs(r_output_dir)

if not os.path.exists(l_output_dir):
    os.makedirs(l_output_dir)

if not os.path.exists(rl_output_dir):
    os.makedirs(rl_output_dir)

if not os.path.exists(r_raw_output_dir):
    os.makedirs(r_raw_output_dir)

if not os.path.exists(l_raw_output_dir):
    os.makedirs(l_raw_output_dir)

# Open bag file
bag = rosbag.Bag(bag_file)
l_ct = r_ct = 0
bridge = CvBridge()
# Loop through bag file and extract images
for topic, msg, t in bag.read_messages(topics=["/camera5/infra1/image_rect", "/camera5/infra2/image_rect", "/camera1/infra1/image_rect_raw", "/camera1/infra2/image_rect_raw", "/camera5/infra1/image_rect_raw", "/camera5/infra2/image_rect_raw"]):
    # Extract image from message
    cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

    # Construct filename for image
    if 'camera1/infra1' in topic:
        # left
        filename = "image_{}_l.jpg".format(t.to_nsec())
        cv2.imwrite(os.path.join(rl_output_dir, filename), cv_img)
        continue
    elif 'camera1/infra2' in topic:
        # right
        filename = "image_{}_r.jpg".format(t.to_nsec())
        cv2.imwrite(os.path.join(rl_output_dir, filename), cv_img)
        continue

    if topic == '/camera5/infra1/image_rect':
        # left
        filename = "image_{}_l.jpg".format(t.to_nsec())
        cv2.imwrite(os.path.join(l_output_dir, filename), cv_img)
    elif topic == '/camera5/infra2/image_rect':
        # right
        filename = "image_{}_r.jpg".format(t.to_nsec())
        cv2.imwrite(os.path.join(r_output_dir, filename), cv_img)

    # if topic == '/camera5/infra1/image_rect_raw':
    #     # left
    #     filename = "image_{}_l.jpg".format(t.to_nsec())
    #     cv2.imwrite(os.path.join(l_raw_output_dir, filename), cv_img)
    # elif topic == '/camera5/infra2/image_rect_raw':
    #     # right
    #     filename = "image_{}_r.jpg".format(t.to_nsec())
    #     cv2.imwrite(os.path.join(r_raw_output_dir, filename), cv_img)


    # Save image to output directory

# Close bag file
bag.close()