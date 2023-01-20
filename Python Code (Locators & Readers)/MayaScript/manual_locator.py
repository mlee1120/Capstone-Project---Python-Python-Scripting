"""
File: manual_locator.py
Description: joint locator - capstone project
Language: python3.7
Author: Michael Lee   ml3406@rit.edu
"""

import os
import cv2
import sys

# number of keyframes
keyframes = None

# keyframe interval
interval = None

# input image
img = None

# x position of the mouse click
mouseX = None

# y position of the mouse click
mouseY = None

# front view or side view
front = None


def inquire(directory):
    '''
    This function sets up the number of keyframes and ask users to input the keyframe interval.

    :param directory: the path of folder which contains all input data
    '''
    global keyframes, interval

    # set up keyframes
    keyframes = 0
    for path in os.listdir(directory + "\\front"):
        if os.path.isfile(os.path.join(directory + "\\front", path)):
            keyframes += 1

    # ask for keyframe interval
    print("Keyframe interval: ", end="")
    interval = input()
    while not interval.isnumeric() or int(interval) <= 0:
        print("Please input an integer larger than 0.")
        print("Keyframe interval: ", end="")
        interval = input()
    interval = int(interval)
    print("There are " + str(keyframes) + " keyframes.")
    print("The keyframe interval is " + str(interval) + ".")


def locate(directory, filename):
    '''
    This function helps users to locate joint position and store all infomation in a text file.

    :param directory: the path of folder which contains all input data
    :param filename: the output filename
    '''
    global img, front

    # a list of all joints
    joints = ["hips", "right_upper_leg", "right_lower_leg", "right_foot", "right_toes", "left_upper_leg",
              "left_lower_leg", "left_foot", "left_toes", "spine", "chest", "upper_chest", "right_shoulder",
              "right_upper_arm", "right_lower_arm", "right_hand", "left_shoulder", "left_upper_arm" "left_lower_arm",
              "left_hand", "neck", "head", "jaw"]
    front = True

    with open(filename, 'w') as file:
        file.write("Keyframes: " + str(keyframes) + "\n")
        file.write("Interval: " + str(interval) + "\n")
        for i in range(len(joints)):
            file.write(joints[i] + " ")
        file.write("\n")

        for i in range(keyframes):
            print("keyframe: " + str(i + 1) + "/" + str(keyframes))
            for j in range(len(joints)):
                print("\nCurrent joint: " + joints[j])
                print("xy positions...")
                img = cv2.imread(directory + "\\front\\front (" + str(i * interval + 1) + ").jpg")
                cv2.imshow("image", img)
                cv2.setMouseCallback("image", left_click_event)
                cv2.waitKey(0)
                file.write(str(mouseX) + " " + str(img.shape[0] - mouseY) + " ")
                front = not front
                print("z position...")
                img = cv2.imread(directory + "\\side\\side (" + str(i * interval + 1) + ").jpg")
                cv2.imshow("image", img)
                cv2.setMouseCallback("image", left_click_event)
                cv2.waitKey(0)
                file.write(str(img.shape[1] - mouseX) + " ")
                front = not front
            file.write("\n")


def left_click_event(event, x, y, flags, params):
    global mouseX, mouseY

    # checking for left mouse clicks
    if event == cv2.EVENT_LBUTTONDOWN:
        # print out the coordinate
        mouseX = x
        mouseY = y
        if front:
            print("x = " + str(mouseX) + ", y = " + str(mouseY))
        else:
            print("z = " + str(mouseX))
        print("Press any key to comfirm or click on the current joint again.")


'''
main conditional guard
The following condition checks whether we are running as a script.
If the file is being imported, don't run the test code.
'''
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: manual_locator.py input_file_pathname output_filename")
        print("Input file path folder should contains a folder called 'front' which contains all front-view "
              + "keyframes and another folder called 'side' which contains all side-view keyframes.")
        sys.exit(-1)
    else:
        inquire(sys.argv[1])
        locate(sys.argv[1], sys.argv[2])
