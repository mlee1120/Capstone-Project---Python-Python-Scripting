"""
File: 04_keypoints_from_images.py
Description: OpenPose joint locator - capstone project
Language: python3.7
Auther: OpenPose team
Editor: Michael Lee   ml3406@rit.edu
"""

import sys
import cv2
import os
import argparse
import time

if len(sys.argv) != 3:
    print("Usage: 04_keypoints_from_images.py input_directory input_motion_name")
    sys.exit(-1)

# front-view frames
path = sys.argv[1] + "/" + sys.argv[2] + "/front"

# set up the number of frames
frames = 0
for p in os.listdir(path):
    if os.path.isfile(os.path.join(path, p)):
        frames += 1

try:
    # Import Openpose (Windows/Ubuntu/OSX)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    try:
        # Change these variables to point to the correct folder (Release/x64 etc.)
        sys.path.append(dir_path + '/../bin/python/openpose/Release')
        os.environ['PATH'] = os.environ['PATH'] + ';' + dir_path + '/../x64/Release;' + dir_path + '/../bin;'
        import pyopenpose as op
    except ImportError as e:
        print(
            'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python '
            'script in the right folder?')
        raise e

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_dir", default=path,
                        help="Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).")
    parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "../models/"

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1]) - 1:
            next_item = args[1][i + 1]
        else:
            next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-', '')
            if key not in params:
                params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-', '')
            if key not in params:
                params[key] = next_item

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    # Read frames on directory
    imagePaths = op.get_images_on_directory(args[0].image_dir)
    start = time.time()

    # save joint positions from the front view
    with open(sys.argv[2] + "-front.txt", 'w') as file:
        file.write("Frames: " + str(frames) + "\n")
        file.write("Scale_Factor: 150\n")
        file.write("head upper_chest right_upper_arm right_lower_arm right_hand left_upper_arm left_lower_arm "
                   "left_hand hips right_upper_leg right_lower_leg right_ankle left_upper_leg left_lower_leg "
                   "left_ankle right_eye left_eye right_ear left_ear left_toes left_toes2 left_heel right_toes "
                   "right_toes2 right_heel\n")

        # Process and display images
        for imagePath in imagePaths:
            datum = op.Datum()
            imageToProcess = cv2.imread(imagePath)
            datum.cvInputData = imageToProcess
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            for i in range(25):
                file.write(str(round(datum.poseKeypoints[0][i][0], 3)) + " ")
                file.write(str(round(datum.poseKeypoints[0][i][1], 3)) + " ")
            file.write("\n")

            if not args[0].no_display:
                cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
                key = cv2.waitKey(15)
                if key == 27:
                    break

    end = time.time()
    print("OpenPose demo successfully finished. Total time: " + str(end - start) + " seconds")
except Exception as e:
    print(e)
    sys.exit(-1)

# side-view frames
path = sys.argv[1] + "/" + sys.argv[2] + "/side"

try:
    # Import Openpose (Windows/Ubuntu/OSX)
    dir_path = os.path.dirname(os.path.realpath(__file__))
    try:
        # Change these variables to point to the correct folder (Release/x64 etc.)
        sys.path.append(dir_path + '/../bin/python/openpose/Release')
        os.environ['PATH'] = os.environ['PATH'] + ';' + dir_path + '/../x64/Release;' + dir_path + '/../bin;'
        import pyopenpose as op
    except ImportError as e:
        print(
            'Error: OpenPose library could not be found. Did you enable `BUILD_PYTHON` in CMake and have this Python '
            'script in the right folder?')
        raise e

    # Flags
    parser = argparse.ArgumentParser()
    parser.add_argument("--image_dir", default=path,
                        help="Process a directory of images. Read all standard formats (jpg, png, bmp, etc.).")
    parser.add_argument("--no_display", default=False, help="Enable to disable the visual display.")
    args = parser.parse_known_args()

    # Custom Params (refer to include/openpose/flags.hpp for more parameters)
    params = dict()
    params["model_folder"] = "../models/"

    # Add others in path?
    for i in range(0, len(args[1])):
        curr_item = args[1][i]
        if i != len(args[1]) - 1:
            next_item = args[1][i + 1]
        else:
            next_item = "1"
        if "--" in curr_item and "--" in next_item:
            key = curr_item.replace('-', '')
            if key not in params:
                params[key] = "1"
        elif "--" in curr_item and "--" not in next_item:
            key = curr_item.replace('-', '')
            if key not in params:
                params[key] = next_item

    # Starting OpenPose
    opWrapper = op.WrapperPython()
    opWrapper.configure(params)
    opWrapper.start()

    # Read frames on directory
    imagePaths = op.get_images_on_directory(args[0].image_dir)
    start = time.time()

    # save joint positions from the side view
    with open(sys.argv[2] + "-side.txt", 'w') as file:
        # Process and display images
        for imagePath in imagePaths:
            datum = op.Datum()
            imageToProcess = cv2.imread(imagePath)
            datum.cvInputData = imageToProcess
            opWrapper.emplaceAndPop(op.VectorDatum([datum]))
            for i in range(25):
                file.write(str(round(datum.poseKeypoints[0][i][0], 3)) + " ")
            file.write("\n")

            if not args[0].no_display:
                cv2.imshow("OpenPose 1.7.0 - Tutorial Python API", datum.cvOutputData)
                key = cv2.waitKey(15)
                if key == 27:
                    break

    end = time.time()
    print("OpenPose demo successfully finished. Total time: " + str(end - start) + " seconds")
except Exception as e:
    print(e)
    sys.exit(-1)
