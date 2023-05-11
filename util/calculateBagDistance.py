#!/usr/bin/env python

import rosbag
import argparse
from nav_msgs.msg import Odometry
import os


def odometry_callback(msg, prev_msg):
    global total_distance
    position = msg.pose.pose.position
    prev_position = prev_msg.pose.pose.position
    distance = ((position.x - prev_position.x) ** 2 +
                (position.y - prev_position.y) ** 2) ** 0.5
    total_distance += distance

def calculate_distance(bag_file):
    global total_distance
    total_distance = 0.0
    prev_msg = None
    count = 0
    with rosbag.Bag(bag_file, 'r') as bag:
        for topic, msg, t in bag.read_messages(topics=['/burro_base/odom']):
            if prev_msg is None:
                prev_msg = msg
                continue
            odometry_callback(msg, prev_msg)
            prev_msg = msg
            count += 1
    return total_distance

if __name__ == '__main__':
    '''
    The script takes the path to a bag and calcuate the distance traveled with base odom
    '''

    parser = argparse.ArgumentParser(description='Distance Calculator')
    parser.add_argument('-p', '--bag_path', type=str, nargs=1, required=False,
                        help='absolute path to a bag')
    args = parser.parse_args()

    if args.bag_path is None:
        bagdir = os.environ['BAGDIR'] + '/' + os.environ['BAGFILE']
        bag_filename = os.environ['BAGFILE'] + '.bag'
        bagfile = bagdir + '/' + bag_filename
    else:
        bagfile = args.bag_path[0]

    distance = calculate_distance(bagfile)
    print('Total traveled distance: {:.2f} meters'.format(distance))

