#!/usr/bin/env python

import yaml
import argparse
from nav_msgs.msg import Odometry
import subprocess

if __name__ == '__main__':
    '''
    Two ways of running the scripts
        1)./distance.py -p <absolute_path>
        OR
        2) bagconfig, then ./distance.py
    '''
    parser = argparse.ArgumentParser(description='extract odom and measure distance')
    parser.add_argument('-c', '--config', type=str, nargs=1, required=True,
                        help='path to the config file')
    args = parser.parse_args()

    with open(args.config[0], 'r') as f:
        while True:

            # Get next line from file
            line = f.readline()
            bag_name = line.strip(" \n")
            # if line is empty
            # end of file is reached
            if not line:
                break

            # extract bag with the topics, some bags are too big to load
            print(f"\nextract {bag_name}")
            output_bag = './' + bag_name.replace('/','-')[1:-4] + '.bag'
            topics = "topic=='/burro_base/odom' or topic=='/gps_rover/fix'"
            cmd_input = "rosbag filter " + bag_name + ' ' + output_bag
            input_list = cmd_input.split()
            input_list.append(topics)
            subprocess.run(input_list, text=True)

            # calculate the distance of a bag given absolute path to a bag
            print(f"Total Distance for bag {output_bag} : {bag_name}")
            cmd_input = "./calculateBagDistance.py -p " + output_bag
            subprocess.run(cmd_input.split(), text=True)
