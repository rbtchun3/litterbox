#!/usr/bin/env python3

import numpy as np
import cv2
import os
import argparse

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("-r", "--realsense", action='store_true')

    args = parser.parse_args()
    if args.realsense == True:
      l_dir = "/home/chun/Downloads/20230216_130118_archive/RL_L"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/RL_R"
    else:
      l_dir = "/home/chun/Downloads/20230216_130118_archive/L"
      r_dir = "/home/chun/Downloads/20230216_130118_archive/R"
    # Set directory path


    # Get list of files in directory
    l_files = os.listdir(l_dir)
    r_files = os.listdir(r_dir)

    # Sort files in alphabetical order
    l_files.sort()
    r_files.sort()

    # # Print list of files in alphabetical order
    # print(l_files)
    # print(r_files)

    # Reading the mapping values for stereo image rectification
    # cv_file = cv2.FileStorage("data/stereo_rectify_maps.xml", cv2.FILE_STORAGE_READ)
    # Left_Stereo_Map_x = cv_file.getNode("Left_Stereo_Map_x").mat()
    # Left_Stereo_Map_y = cv_file.getNode("Left_Stereo_Map_y").mat()
    # Right_Stereo_Map_x = cv_file.getNode("Right_Stereo_Map_x").mat()
    # Right_Stereo_Map_y = cv_file.getNode("Right_Stereo_Map_y").mat()
    # cv_file.release()

    def nothing(x):
        pass

    cv2.namedWindow('disp',cv2.WINDOW_NORMAL)
    cv2.resizeWindow('disp',600,600)

    cv2.createTrackbar('numDisparities','disp',16,128,nothing)
    cv2.createTrackbar('blockSize','disp',5,50,nothing)
    cv2.createTrackbar('preFilterType','disp',1,1,nothing)
    cv2.createTrackbar('preFilterSize','disp',9,25,nothing)
    cv2.createTrackbar('preFilterCap','disp',5,62,nothing)
    cv2.createTrackbar('textureThreshold','disp',10,100,nothing)
    cv2.createTrackbar('uniquenessRatio','disp',15,100,nothing)
    cv2.createTrackbar('speckleRange','disp',0,100,nothing)
    cv2.createTrackbar('speckleWindowSize','disp',3,100,nothing)
    cv2.createTrackbar('disp12MaxDiff','disp',-1,25,nothing)
    cv2.createTrackbar('minDisparity','disp',0,64,nothing)



    # Creating an object of StereoBM algorithm
    stereo = cv2.StereoBM_create()


    # # Setting parameters for StereoSGBM algorithm
    # minDisparity = 8
    # numDisparities = 64
    # blockSize = 1
    # disp12MaxDiff = 1
    # uniquenessRatio = 10
    # speckleWindowSize = 10
    # speckleRange = 8

    #     # Creating an object of StereoSGBM algorithm
    # stereo2 = cv2.StereoSGBM_create(minDisparity = minDisparity,
    #         numDisparities = numDisparities,
    #         blockSize = blockSize,
    #         disp12MaxDiff = disp12MaxDiff,
    #         uniquenessRatio = uniquenessRatio,
    #         speckleWindowSize = speckleWindowSize,
    #         speckleRange = speckleRange
    #     )


    block_size = 11
    min_disp = -128
    max_disp = 128
    # Maximum disparity minus minimum disparity. The value is always greater than zero.
    # In the current implementation, this parameter must be divisible by 16.
    num_disp = max_disp - min_disp
    # Margin in percentage by which the best (minimum) computed cost function value should "win" the second best value to consider the found match correct.
    # Normally, a value within the 5-15 range is good enough
    uniquenessRatio = 5
    # Maximum size of smooth disparity regions to consider their noise speckles and invalidate.
    # Set it to 0 to disable speckle filtering. Otherwise, set it somewhere in the 50-200 range.
    speckleWindowSize = 200
    # Maximum disparity variation within each connected component.
    # If you do speckle filtering, set the parameter to a positive value, it will be implicitly multiplied by 16.
    # Normally, 1 or 2 is good enough.
    speckleRange = 2
    disp12MaxDiff = 0

    stereo2 = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=block_size,
        uniquenessRatio=uniquenessRatio,
        speckleWindowSize=speckleWindowSize,
        speckleRange=speckleRange,
        disp12MaxDiff=disp12MaxDiff,
        P1=8 * 1 * block_size * block_size,
        P2=32 * 1 * block_size * block_size,
    )


    # Calculating disparith using the StereoSGBM algorithm

    ct = 0
    while True:

      if ct >= len(r_files):
        break
      # Capturing and storing left and right camera images
      retL = retR = True

      # Proceed only if the frames have been captured
      if retL and retR:
        imgR_gray = cv2.imread(os.path.join(r_dir, r_files[ct]))
        imgL_gray = cv2.imread(os.path.join(l_dir, l_files[ct]))

        # # Applying stereo image rectification on the left image
        # Left_nice= cv2.remap(imgL_gray,
        #           Left_Stereo_Map_x,
        #           Left_Stereo_Map_y,
        #           cv2.INTER_LANCZOS4,
        #           cv2.BORDER_CONSTANT,
        #           0)

        # # Applying stereo image rectification on the right image
        # Right_nice= cv2.remap(imgR_gray,
        #           Right_Stereo_Map_x,
        #           Right_Stereo_Map_y,
        #           cv2.INTER_LANCZOS4,
        #           cv2.BORDER_CONSTANT,
        #           0)

        Left_nice, Right_nice = cv2.cvtColor(imgL_gray, cv2.COLOR_BGR2GRAY), cv2.cvtColor(imgR_gray, cv2.COLOR_BGR2GRAY)

        # Updating the parameters based on the trackbar positions
        numDisparities = cv2.getTrackbarPos('numDisparities','disp')
        blockSize = cv2.getTrackbarPos('blockSize','disp')
        preFilterType = cv2.getTrackbarPos('preFilterType','disp')
        preFilterSize = cv2.getTrackbarPos('preFilterSize','disp')
        preFilterCap = cv2.getTrackbarPos('preFilterCap','disp')
        textureThreshold = cv2.getTrackbarPos('textureThreshold','disp')
        uniquenessRatio = cv2.getTrackbarPos('uniquenessRatio','disp')
        speckleRange = cv2.getTrackbarPos('speckleRange','disp')
        speckleWindowSize = cv2.getTrackbarPos('speckleWindowSize','disp')
        disp12MaxDiff = cv2.getTrackbarPos('disp12MaxDiff','disp')
        minDisparity = cv2.getTrackbarPos('minDisparity','disp')

        # Setting the updated parameters before computing disparity map
        stereo.setNumDisparities(numDisparities)
        stereo.setBlockSize(blockSize)
        stereo.setPreFilterType(preFilterType)
        stereo.setPreFilterSize(preFilterSize)
        stereo.setPreFilterCap(preFilterCap)
        stereo.setTextureThreshold(textureThreshold)
        stereo.setUniquenessRatio(uniquenessRatio)
        stereo.setSpeckleRange(speckleRange)
        stereo.setSpeckleWindowSize(speckleWindowSize)
        stereo.setDisp12MaxDiff(disp12MaxDiff)
        stereo.setMinDisparity(minDisparity)

        # stereo2.setNumDisparities(numDisparities)
        # stereo2.setBlockSize(blockSize)
        # stereo2.setUniquenessRatio(uniquenessRatio)
        # stereo2.setSpeckleRange(speckleRange)
        # stereo2.setSpeckleWindowSize(speckleWindowSize)
        # stereo2.setDisp12MaxDiff(disp12MaxDiff)
        # stereo2.setMinDisparity(minDisparity)


        # Calculating disparity using the StereoBM algorithm
        disparity = stereo.compute(Left_nice, Right_nice)
        # NOTE: Code returns a 16bit signed single channel image,
        # CV_16S containing a disparity map scaled by 16. Hence it
        # is essential to convert it to CV_32F and scale it down 16 times.

        # Converting to float32
        disparity = disparity.astype(np.float32)

        # Scaling down the disparity values and normalizing them
        disparity = (disparity/16.0 - minDisparity)/numDisparities
        # disparity = cv2.normalize(disparity,0,255,cv2.NORM_MINMAX)

        print(f"type: {type(disparity)} with shape {disparity.shape} with range from {np.min(disparity)} to {np.max(disparity)}")

        # Displaying the disparity map
        cv2.imshow("disp",disparity)
        cv2.waitKey(0)

        ct += 1
        # Close window using esc key
        if cv2.waitKey(1) == 27:
          return

if __name__ == "__main__":
    main()