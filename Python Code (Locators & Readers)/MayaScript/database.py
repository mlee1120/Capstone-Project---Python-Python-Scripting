"""
File: database.py
Description: database_scripting_maya - capstone project
Language: python3.7
Author: Michael Lee   ml3406@rit.edu
"""

import os
import sys

# a list to store tuples having a joint with an index for looking up motion data
data_index = list()

# a list to store motion data
data = list()

# number of frames
frames = 0


def read_data(filename):
    """
    This function reads .bvh files and store the motion data in corresponding data sturctures.

    :param filename: the input filename
    """

    global frames, data, data_index
    try:
        with open(filename) as file:
            # finish reading hierarchy or not
            endHierarchy = False
            end = 0
            # joint's index for their motion data
            index = 0

            # read hierarchy and find corresponding joints in my model (mapping)
            while end != 0 or not endHierarchy:
                line = file.readline()
                line.strip()
                splitLine = line.split()
                if splitLine[0].lower() == "hierarchy":
                    pass
                elif splitLine[0].lower() == "root":
                    if splitLine[1].lower() == "hips":
                        endHierarchy = True
                        line = file.readline()
                        line = file.readline()
                        line = file.readline()
                        data_index.append(("hips", index))
                        index = 2
                        end = 1
                elif splitLine[0].lower() == "{":
                    end += 1
                elif splitLine[0].lower() == "offset":
                    pass
                elif splitLine[0].lower() == "channels":
                    pass
                elif splitLine[0].lower() == "joint":
                    if splitLine[1].lower() == "lefthip":
                        data_index.append(("upper_leg_left", index))
                        index += 1
                    elif splitLine[1].lower() == "leftknee":
                        data_index.append(("lower_leg_left", index))
                        index += 1
                    elif splitLine[1].lower() == "leftankle":
                        data_index.append(("foot_left", index))
                        index += 1
                    elif splitLine[1].lower() == "righthip":
                        data_index.append(("upper_leg_right", index))
                        index += 1
                    elif splitLine[1].lower() == "rightknee":
                        data_index.append(("lower_leg_right", index))
                        index += 1
                    elif splitLine[1].lower() == "rightankle":
                        data_index.append(("foot_right", index))
                        index += 1
                    elif splitLine[1].lower() == "chest":
                        data_index.append(("chest", index))
                        index += 1
                    elif splitLine[1].lower() == "leftcollar":
                        data_index.append(("shoulder_left", index))
                        index += 1
                    elif splitLine[1].lower() == "leftshoulder":
                        data_index.append(("upper_arm_left", index))
                        index += 1
                    elif splitLine[1].lower() == "leftelbow":
                        data_index.append(("lower_arm_left", index))
                        index += 1
                    elif splitLine[1].lower() == "leftwrist":
                        index += 1
                    elif splitLine[1].lower() == "rightcollar":
                        data_index.append(("shoulder_right", index))
                        index += 1
                    elif splitLine[1].lower() == "rightshoulder":
                        data_index.append(("upper_arm_right", index))
                        index += 1
                    elif splitLine[1].lower() == "rightelbow":
                        data_index.append(("lower_arm_right", index))
                        index += 1
                    elif splitLine[1].lower() == "rightwrist":
                        index += 1
                    elif splitLine[1].lower() == "neck":
                        data_index.append(("neck", index))
                        index += 1
                    elif splitLine[1].lower() == "head":
                        data_index.append(("head", index))
                        index += 1
                elif splitLine[0].lower() == "end":
                    pass
                elif splitLine[0].lower() == "}":
                    end -= 1

            # read and save motion data
            while True:
                line = file.readline()
                line.strip()
                splitLine = line.split()
                if splitLine[0].lower() == "motion":
                    break
            line = file.readline()
            line.strip()
            splitLine = line.split()
            frames = int(splitLine[1])
            line = file.readline()
            for i in range(frames):
                line = file.readline()
                line.strip()
                splitLine = line.split()
                data.append(list())
                for number in splitLine:
                    data[-1].append(float(number))
    except:
        print(os.getcwd() + filename + " may not exist.\nPlease check and try again.")


def write_data(filename):
    """
    This function converts motion data to Python sciprting in Maya.

    :param filename: the output filename
    """

    # a list of all joints
    joints = ["hips", "upper_leg_right", "lower_leg_right", "foot_right", "toes_right", "upper_leg_left",
              "lower_leg_left", "foot_left", "toes_left", "spine", "chest", "upper_chest", "shoulder_right",
              "upper_arm_right", "lower_arm_right", "hand_right", "thumb_proximal_right", "thumb_intermediate_right",
              "thumb_distal_right", "index_proximal_right", "index_intermediate_right", "index_distal_right",
              "middle_proximal_right", "middle_intermediate_right", "middle_distal_right", "ring_proximal_right",
              "ring_intermediate_right", "ring_distal_right", "little_proximal_right", "little_intermediate_right",
              "little_distal_right", "shoulder_left", "upper_arm_left", "lower_arm_left", "hand_left",
              "thumb_proximal_left", "thumb_intermediate_left", "thumb_distal_left", "index_proximal_left",
              "index_intermediate_left", "index_distal_left", "middle_proximal_left", "middle_intermediate_left",
              "middle_distal_left", "ring_proximal_left", "ring_intermediate_left", "ring_distal_left",
              "little_proximal_left", "little_intermediate_left", "little_distal_left", "neck", "head", "eye_right",
              "eye_left", "jaw"]

    # a factor that scales the traslations in motion data
    f = 36.0

    # auxiliary variables for setting initial hips' position to (0.0, 0.0, 0.0)
    dx = 0.0
    dy = 0.0
    dz = 0.0

    with open(filename, 'w') as file:
        file.write("# import module that contains most Maya MEL commands to Python\n")
        file.write("import maya.cmds as cmds\n\n")
        file.write("# delete keysframes of all joints\n")
        for joint in joints:
            file.write("cmds.cutKey('" + joint + "', time=(1, " + str(frames) + "))\n")
        file.write("\n# set keyframes\n")
        for i in range(frames):
            for joint in data_index:
                if joint[1] == 0:
                    if dx == 0.0:
                        dx = data[i][joint[1] * 3]
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='translateX', value=" + str((data[i][joint[1] * 3] - dx) / f) + ")\n")
                    if dy == 0.0:
                        dy = data[i][joint[1] * 3 + 1]
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='translateY', value=" + str(
                        (data[i][joint[1] * 3 + 1] - dy) / f + 4.6) + ")\n")
                    if dz == 0.0:
                        dz = data[i][joint[1] * 3 + 2]
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='translateZ', value=" + str((data[i][joint[1] * 3 + 2] - dz) / f) + ")\n")
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateZ', value=" + str(data[i][joint[1] * 3 + 3]) + ")\n")
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateX', value=" + str(data[i][joint[1] * 3 + 4]) + ")\n")
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateY', value=" + str(data[i][joint[1] * 3 + 5]) + ")\n")
                else:
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateZ', value=" + str(data[i][joint[1] * 3]) + ")\n")
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateX', value=" + str(data[i][joint[1] * 3 + 1]) + ")\n")
                    file.write("cmds.setKeyframe('" + joint[0] + "', time=" + str(i + 1) +
                               ", attribute='rotateY', value=" + str(data[i][joint[1] * 3 + 2]) + ")\n")


'''
main conditional guard
The following condition checks whether we are running as a script.
If the file is being imported, don't run the test code.
'''
if __name__ == '__main__':
    if len(sys.argv) != 3:
        print("Usage: database.py input_file_pathname output_filename")
        sys.exit(-1)
    else:
        read_data(sys.argv[1])
        write_data(sys.argv[2])
