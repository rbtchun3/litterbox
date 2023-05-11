#!/usr/bin/python

import yaml
import sys
import cv2
import cv_bridge
import numpy as np
import os
import rosbag, rospy

class Bag2Video(object):
    def __init__(self, bag_name, topic_name, **kwargs):
        self.bag_name = bag_name
        self.topic_name = topic_name

        self.codec = kwargs.get('codec', 'MJPG')
        self.time_step = kwargs.get('time_step', 0.5) #Set to 0 in order to not skip frames #prev_val=0.5
        self.fps = kwargs.get('fps', 10) #Set to 15fps for Bitwise #prev_val=10
        container = kwargs.get('container', 'mp4')

        self.bridge = cv_bridge.CvBridge()
        self.vwriter = None

        directory = '/home/chun'
        bag_id = os.path.basename(bag_name).split('.')[0]

        topic = topic_name[1:].replace('/','-')
        video_name = "{}-{}.{}".format(bag_id, topic, container)
        self.video_full_path = os.path.join(directory, video_name)

        # print 'Creating video output ({})'.format(self.video_full_path)

    def create(self, bag=None):
        if bag is None:
            self.bag = rosbag.Bag(self.bag_name, 'r')
        else:
            self.bag = bag

        if self.exists() == True:
            return True

        last_time = rospy.Time(0)
        for topic, msg, t in self.bag.read_messages(topics=[self.topic_name]):
            if (t - last_time).to_sec() < self.time_step:
                continue

            last_time = t
            if msg._type == "sensor_msgs/Image":
                img = self.bridge.imgmsg_to_cv2(msg, "passthrough")
            elif msg._type == "sensor_msgs/CompressedImage":
                np_arr = np.fromstring(msg.data, np.uint8)
                img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            else:
                # print "Message of type {} is not an image".format(msg._type)
                return False
            if self.vwriter is None:
                # print 'Writing frames...'
                self.vwriter = cv2.VideoWriter(self.video_full_path,
                                               cv2.VideoWriter_fourcc(*self.codec),
                                               self.fps,
                                               (img.shape[1], img.shape[0]))

            if len(img.shape) == 3:
                self.vwriter.write(cv2.cvtColor(img, cv2.COLOR_BGR2RGB))
            else:
                self.vwriter.write(cv2.cvtColor(img, cv2.COLOR_GRAY2RGB))

        return self.exists()

    def exists(self):
        return os.path.exists(self.video_full_path)

    def __del__(self):
        if self.vwriter is not None:
            self.vwriter.release()


if __name__ == "__main__":
    topic_name = '/camera4/infra1/image_rect_raw'
    file1 = open('./archive_bag_path.txt', 'r')

    '''
    get list of all archive bags by doing this on hercules     find /hdd*/*bag*/ -name '*archive.bag' > somefile.txt
    loop through all archive bags
        get conversion names
        check if conversion video exists
         if does
            mark complete in yaml
         else
            try conversion
                check if conversion success
            mark 'unindexed' if catching errors
            mark '???' if no conversion and no errors
            mark 'ok' if nothing above is fitted
    '''

    bag_status = {}
    while True:

        # Get next line from file
        line = file1.readline()
        bag_name = line.strip(" \n")
        # if line is empty
        # end of file is reached
        if not line:
            break
        try:
            bag2video = Bag2Video(bag_name[1:], topic_name)
            if bag2video.create():
                bag_status[bag_name[1:]] = 'OK' # exists or conversion done, Q: reverse search from a video to a bag
            else:
                bag_status[bag_name[1:]] = '???' # not sure why conversion didn't happen
        except Exception as err:
            bag_status[bag_name[1:]] = str(err)
            # print(f"Could not convert {bag_name} with error: {err}")
        print(f"conversion of bag {bag_name[1:]} is {bag_status[bag_name[1:]]}")
    file1.close()

    with open('./bag_status.yaml', 'w') as f:
        yaml.dump(bag_status, f, default_flow_style=False)