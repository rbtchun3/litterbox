#!/usr/bin/env python3


import numpy as np
import cv2 as cv
import matplotlib.pyplot as plt
import os, argparse


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--realsense", action='store_true')
    npatchsize = 31
    fast_threshold = 20
    args = parser.parse_args()
    if args.realsense == True:
      # Set directory path
      l_dir = "/home/chun/Downloads/20230216_130118_archive/RL_L"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/RL_R"
    else:
      l_dir = "/home/chun/Downloads/20230216_130118_archive/L"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/R"
      npatchsize = 62
      fast_threshold = 5

    # Get list of files in directory
    l_files = os.listdir(l_dir)
    r_files = os.listdir(r_dir)

    # Sort files in alphabetical order
    l_files.sort()
    r_files.sort()

    ct = 0
    while True:

        if ct >= len(r_files):
            break

        img1 = cv.imread(os.path.join(l_dir, l_files[ct]),cv.IMREAD_GRAYSCALE) # queryImage
        img2 = cv.imread(os.path.join(r_dir, r_files[ct]),cv.IMREAD_GRAYSCALE) # trainImage
        # Initiate SIFT detector
        # sift = cv.SIFT_create()
        # # find the keypoints and descriptors with SIFT
        # kp1, des1 = sift.detectAndCompute(img1,None)
        # kp2, des2 = sift.detectAndCompute(img2,None)

        orb = cv.ORB_create(10000, patchSize = npatchsize, fastThreshold=fast_threshold)
        # # find the keypoints with ORB
        kp1 = orb.detect(img1,None)
        kp2 = orb.detect(img2,None)
        # # compute the descriptors with ORB
        kp1, des1 = orb.compute(img1, kp1)
        kp2, des2 = orb.compute(img2, kp2)

        # # BFMatcher with default params
        bf = cv.BFMatcher(cv.NORM_HAMMING, crossCheck=True)
        matches = bf.match(des1,des2)

        # kp2 = []
        # fast = cv.FastFeatureDetector_create(threshold=10)
        # # find and draw the keypoints
        # print(f"image size {img1.shape}")
        # kp1 = fast.detect(img1,None)
        # img2 = cv.drawKeypoints(img1, kp1, None, color=(255,0,0))
        # cv.imshow("disp",img2)
        # Print all default params
        # print( "Threshold: {}".format(fast.getThreshold()) )
        # print( "nonmaxSuppression:{}".format(fast.getNonmaxSuppression()) )
        # print( "neighborhood: {}".format(fast.getType()) )
        # print( "Total Keypoints with nonmaxSuppression: {}".format(len(kp)) )
        # cv.imwrite('fast_true.png', img2)
        # # Disable nonmaxSuppression
        # fast.setNonmaxSuppression(0)
        # kp = fast.detect(img, None)
        # print( "Total Keypoints without nonmaxSuppression: {}".format(len(kp)) )
        # img3 = cv.drawKeypoints(img, kp, None, color=(255,0,0))
        # cv.imwrite('fast_false.png', img3)

        # Apply ratio test
        # good = []
        # for m,n in matches:
        #     if m.distance < 0.75*n.distance:
        #         good.append([m])


        print(f"num of matches {len(matches)} with number of descriptor on left: {len(kp1)} |||| on right: {len(kp2)}")
        # cv.drawMatchesKnn expects list of lists as matches.
        img3 = cv.drawMatches(img1,kp1,img2,kp2,matches,None,flags=cv.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)


        cv.imshow("disp",img3)
        ct += 1
        cv.waitKey(0)

if __name__ == "__main__":
    '''
    The script does the left and right mono camera matching with Orb
    '''
    main()