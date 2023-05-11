#!/usr/bin/env python3

import numpy as np
import cv2
import os
import rospy
import argparse
import matplotlib.pyplot as plt

def vconcat_resize_min(im_list, interpolation=cv2.INTER_CUBIC):
    w_min = min(im.shape[1] for im in im_list)
    im_list_resize = [cv2.resize(im, (w_min, int(im.shape[0] * w_min / im.shape[1])), interpolation=interpolation)
                      for im in im_list]
    return cv2.vconcat(im_list_resize)

def hconcat_resize_min(im_list, interpolation=cv2.INTER_CUBIC):
    h_min = min(im.shape[0] for im in im_list)
    im_list_resize = [cv2.resize(im, (int(im.shape[1] * h_min / im.shape[0]), h_min), interpolation=interpolation)
                      for im in im_list]
    return cv2.hconcat(im_list_resize)

def concat_tile_resize(im_list_2d, interpolation=cv2.INTER_CUBIC):
    im_list_v = [hconcat_resize_min(im_list_h, interpolation=cv2.INTER_CUBIC) for im_list_h in im_list_2d]
    return vconcat_resize_min(im_list_v, interpolation=cv2.INTER_CUBIC)

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--realsense", action='store_true')

    args = parser.parse_args()
    if args.realsense == True:
      l_dir = "/home/chun/Downloads/20230216_130118_archive/RLL"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/RLR"
      d_name = 'realsense'
    else:
      d_name = 'oakd'
      l_dir = "/home/chun/Downloads/20230216_130118_archive/L"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/R"
    # params for corner detection
    feature_params = dict( maxCorners = 5000,
                        qualityLevel = 0.3,
                        minDistance = 12,
                        blockSize = 7 )

    # Parameters for lucas kanade optical flow
    lk_params = dict( winSize = (15, 3),
                    maxLevel = 5,
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT,
                                30, 1e-2),
                    flags = 8,
                    minEigThreshold = 1e-4)



    # Get list of files in directory
    l_files = os.listdir(l_dir)
    r_files = os.listdir(r_dir)

    # Sort files in alphabetical order
    l_files.sort()
    r_files.sort()

    ct = 0

    r_t = [int(r_file.split('_')[1]) for r_file in r_files ]
    hist = [0]*7
    hist_y = [0]*3

    while(1):

        if ct >= len(l_files):
                break

        l_timestamp = int(l_files[ct].split('_')[1])
        f_name = r_files[min(range(len(r_t)), key = lambda i: abs(r_t[i] - l_timestamp))]
        r_timestamp = f_name.split('_')[1]

        print(f"time diff: {abs(rospy.Time(nsecs=l_timestamp) - rospy.Time(nsecs=r_timestamp)) / 1000000.0}")
        old_gray = cv2.imread(os.path.join(l_dir, l_files[ct])) # queryImage
        frame_gray = cv2.imread(os.path.join(r_dir, f_name)) # trainImage
        old_gray_d = cv2.cvtColor(old_gray,cv2.COLOR_BGR2GRAY)

        ct += 1

        orb = cv2.ORB_create(5000, patchSize = 62, fastThreshold=8)
        ## find the keypoints / compute descriptor with ORB
        kp1 = orb.detect(old_gray,None)
        kp1, des1 = orb.compute(old_gray, kp1)
        p0 = cv2.KeyPoint.convert(kp1)
        p0 = np.array([np.array([i]) for i in p0])
        # kp2 = orb.detect(frame_gray,None)
        # kp2, des2 = orb.compute(frame_gray, kp2)

        ## using GFTT
        # p0 = cv2.goodFeaturesToTrack(old_gray_d, mask = None,
        #                         **feature_params)

        mask = np.zeros_like(old_gray)
        whole = np.concatenate((old_gray, frame_gray), axis=1)
        mask2 = np.zeros_like(whole)
        x_offset = old_gray.shape[1]

        # calculate optical flow
        # cv2.calcOpticalFlowPyrLK(prevImg, nextImg, prevPts, nextPts[, winSize[, maxLevel[, criteria]]])'''
        p1, st, err = cv2.calcOpticalFlowPyrLK(old_gray,
                                            frame_gray,
                                            p0, None,
                                            **lk_params)

        # Select good points
        good_new = p1[st == 1]
        good_old = p0[st == 1]

        disparities = []
        y_list = []

        # draw the tracks
        for i, (new, old) in enumerate(zip(good_new,
                                        good_old)):
            a, b = new.ravel()
            c, d = old.ravel()
            y_list.append(abs(b - d))
            if(abs(b - d) > 1): # movement estimated in Y
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)),
                            (255,0,0), 2)
                if (c - a <= 128):
                    mask2 = cv2.line(mask2, (int(a), int(b)), (x_offset + int(c), int(d)),
                                (0,255,255), 1)
                    mask2 = cv2.line(mask2, (int(a), int(b)), (x_offset + int(c), int(b)),
                                (255,255,0), 1)
            elif(c >= a): # valid disparity draw green left.x = c, right.x = a
                disparities.append(c - a)
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)),
                                (0,255, 0), 2)
            else:
                disparities.append(c - a)
                mask = cv2.line(mask, (int(a), int(b)), (int(c), int(d)),
                            (0,0,255), 2)

        dis_np = np.array(disparities)
        y_np = np.array(y_list)

        print(f"<-1000: {np.count_nonzero((dis_np < -1000))}\n\
            -1000 - -100: {np.count_nonzero((dis_np >= -1000) & (dis_np < -100))}\n\
            -100 - -1: {np.count_nonzero((dis_np >= -100) & (dis_np < -1))}\n\
            -1 - 0.: {np.count_nonzero((dis_np >= -1) & (dis_np < 0.0))}\n\
            0 - 0.5: {np.count_nonzero((dis_np >= 0) & (dis_np < 0.5))}\n\
            0.5 - 128: {np.count_nonzero((dis_np < 128) & (dis_np >= 0.5 ))}\n\
            >128: {np.count_nonzero((dis_np >= 128))}\n")

        img = cv2.add(old_gray, mask)
        whole2 = cv2.add(whole, mask2)
        hist[0] += np.count_nonzero((dis_np < -1000))
        hist[1] += np.count_nonzero((dis_np >= -1000) & (dis_np < -100))
        hist[2] += np.count_nonzero((dis_np >= -100) & (dis_np < -1))
        hist[3] += np.count_nonzero((dis_np >= -1) & (dis_np < 0.0))
        hist[4] += np.count_nonzero((dis_np >= 0) & (dis_np < 0.5))
        hist[5] += np.count_nonzero((dis_np < 128) & (dis_np >= 0.5 ))
        hist[6] += np.count_nonzero((dis_np >= 128))

        hist_y[0] += np.count_nonzero((y_np <= 1.0))
        hist_y[1] += np.count_nonzero((y_np > 1) & (y_np <= 100.0))
        hist_y[2] += np.count_nonzero((y_np > 100.0))

        im_tile_resize = concat_tile_resize([[img],
                                            [whole2]])

        cv2.imshow('frame', im_tile_resize)

        k = cv2.waitKey(0)
        if k == 27:
            break

    hist = np.array(hist) / float(ct)
    hist_y = np.array(hist_y) / float(ct)
    cv2.destroyAllWindows()
    plt.bar(np.arange(7), height=hist)
    plt.xticks(np.arange(7), ['<-1000','<-100','<-1','<0', '<0.5', '<128', '>128'])
    plt.title(f'Average Distribution of X Disparity ({ct} of stereo pairs)')
    plt.xlabel('disparity range in x')
    plt.ylabel('frequencies')
    plt.savefig('./distribution_' + d_name + '.png')
    plt.clf()
    plt.cla()
    plt.close()

    plt.bar(np.arange(3), height=hist_y)
    plt.xticks(np.arange(3), ['<=1.0', '<=100.0', '>100'])
    plt.title(f'Average Distribution of Y Disparity ({ct} of stereo pairs)')
    plt.xlabel('disparity range in y')
    plt.ylabel('frequencies')
    plt.savefig('./y_distribution_' + d_name + '.png')

if __name__ == "__main__":
    main()