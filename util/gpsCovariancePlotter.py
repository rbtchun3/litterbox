#!/usr/bin/env python3

import rosbag
import matplotlib.pyplot as plt
import os, subprocess
import numpy as np

def plot_gps(bag_name):
    time_data = []
    topic_data = []

    bag = rosbag.Bag(bag_name)
    topic_name = '/gps_rover/fix'
    plot_name = bag_name[0:-4]

    fix_ct = 0

    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        time_data.append(msg.header.stamp.to_sec())
        topic_data.append(msg.position_covariance[0])
        if msg.status.status == 2:
            fix_ct += 1.0
    ratio = -1

    if len(topic_data) > 0:
        ratio = fix_ct / len(topic_data)

    plt.plot(time_data, topic_data)
    plt.xlabel('Time (s)')
    plt.ylabel('convariance')
    plt.title(topic_name)
    plt.savefig('./' + plot_name + 'fix_ratio_' + str(ratio) + '.png')  # Save the plot as a PNG image file

    mean = np.mean(np.array(topic_data))

    bag.close()


if __name__ == "__main__":
    '''
    The script reads all the bag, calculates the duration covered with GPS fixed status and plots the covariance for each bag
    '''
    dir_files = os.listdir('./')

    for f in dir_files:
        if '.bag' in f:
            cmd_ = 'rosbag reindex ' + f
            try:
                plot_gps(f)
                print(f"plot bag {f}")
            except Exception as err:
                print(f"Could not convert {f} with error: {err}")
                subprocess.run(cmd_.split(), text=True)
                plot_gps(f)
                print(f"retry plot bag {f}")