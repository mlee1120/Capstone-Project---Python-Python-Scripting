"""
File: cv_reader.py
Description: cv_scripting_maya - capstone project
Language: python3.7
Author: Michael Lee   ml3406@rit.edu
"""

import sys
import math


class Quaternion:
    """
    a class represents quaternions
    """

    # instance variables (w, x, y, and z of a quaternion)
    __slots__ = "w", "x", "y", "z"

    def __init__(self, w, x, y, z):
        """
        The constructor initializes all variables.

        :param w: scalar
        :param x: x component
        :param y: y component
        :param z: z component
        """
        self.w = w
        self.x = x
        self.y = y
        self.z = z

    def normalize(self):
        """
        This function normalizes this quaternion.
        """
        wTemp = 1.1
        xTemp = 0.0
        yTemp = 0.0
        zTemp = 0.0
        difference = 0.000000
        while pow(wTemp, 2) + pow(xTemp, 2) + pow(yTemp, 2) + pow(zTemp, 2) > 1.0:
            absolute = (math.sqrt(pow(self.w, 2) + pow(self.x, 2) + pow(self.y, 2) + pow(self.z, 2)) + difference)
            wTemp = self.w / absolute
            xTemp = self.x / absolute
            yTemp = self.y / absolute
            zTemp = self.z / absolute
            difference += 1
        self.w = wTemp
        self.x = xTemp
        self.y = yTemp
        self.z = zTemp

    def inverse(self):
        """
        This function calculates and returns an inverse quaternion of this quaternion.

        :return: an inverse quaternion of this quaternion
        """
        return Quaternion(self.w, -self.x, -self.y, -self.z)

    def product(self, q):
        """
        This function calculates and returns the production of this quaternion and another one.

        :param q: the other quaternion
        :return: the production of the quaternions
        """
        if isinstance(q, Quaternion):
            wTemp = self.w * q.w - self.x * q.x - self.y * q.y - self.z * q.z
            xTemp = self.w * q.x + self.x * q.w + self.y * q.z - self.z * q.y
            yTemp = self.w * q.y - self.x * q.z + self.y * q.w + self.z * q.x
            zTemp = self.w * q.z + self.x * q.y - self.y * q.x + self.z * q.w
            return Quaternion(wTemp, xTemp, yTemp, zTemp)
        else:
            print("The argument of .product() function from Quaternion should be a Quaternion...")
            sys.exit(-1)


class Joint:
    """
    a class represents joints
    """

    '''
    instance variables
    index      - the index of this joint for accessing motion data
    x, y, z    - x, y, and z positions of this joint
    ox, oy, oz - initial x, y, and z positions of this joint
    children   - a list of childten of this joint
    name       - name of this joint
    rotate     - rotate sequence (quaternion/matrix) of this joint
    '''
    __slots__ = "index", "x", "y", "z", "ox", "oy", "oz", "children", "name", "rotate"

    def __init__(self, x, y, z, index, name):
        """
        The constructor initializes all variables.

        :param x: x position
        :param y: y position
        :param z: z position
        :param index: index for accessing motion data
        :param name: name of the joint
        """
        if not isinstance(x, float) or not isinstance(y, float) or \
                not isinstance(z, float) or not isinstance(index, int) or not isinstance(name, str):
            print("Arguments of Joint should be: Joint(float, float, float, int, str)...")
            sys.exit(-1)
        else:
            self.index = index
            self.x = x
            self.y = y
            self.z = z
            self.ox = x
            self.oy = y
            self.oz = z
            self.name = name
            self.children = list()
            self.rotate = None


class Model:
    """
    a class represents models
    """

    # instance variables (root joint of this model)
    __slots__ = "root"


# a model (hierarchy)
myModel = None

# number of frames
frames = 0

# keyframe interval
interval = 0

# a factor that scales the traslations in motion data
factor = 0

# a list to store raw data (joints' positions at every frame)
data = list()

# a list to store motion data
vectors = list()


def set_model():
    """
    This function sets up a model in T-pose.
    """
    global myModel

    myModel = Model()
    joint = None

    # hips
    joint = Joint(0.0, 4.6, 0.0, 0, "hips")
    myModel.root = joint
    # right upper leg
    joint = Joint(-0.45, 4.28, 0.0, 2, "upper_leg_right")
    myModel.root.children.append(joint)
    # right lower leg
    joint = Joint(-0.45, 2.265, 0.0, 3, "lower_leg_right")
    myModel.root.children[0].children.append(joint)
    # right foot
    joint = Joint(-0.45, 0.15, 0.0, 4, "foot_right")
    myModel.root.children[0].children[0].children.append(joint)
    # right toes
    joint = Joint(-0.45, 0.15, 1.0, -1, "toes_right")
    myModel.root.children[0].children[0].children[0].children.append(joint)
    # left upper leg
    joint = Joint(0.45, 4.28, 0.0, 5, "upper_leg_left")
    myModel.root.children.append(joint)
    # left lower leg
    joint = Joint(0.45, 2.265, 0.0, 6, "lower_leg_left")
    myModel.root.children[1].children.append(joint)
    # left foot
    joint = Joint(0.45, 0.15, 0.0, 7, "foot_left")
    myModel.root.children[1].children[0].children.append(joint)
    # left toes
    joint = Joint(0.45, 0.15, 1.0, -1, "toes,left")
    myModel.root.children[1].children[0].children[0].children.append(joint)
    # spine
    joint = Joint(0.0, 5.3, 0.0, -1, "spine")
    myModel.root.children.append(joint)
    # chest
    joint = Joint(0.0, 6.05, 0.0, -1, "chest")
    myModel.root.children[2].children.append(joint)
    # upper chest
    joint = Joint(0.0, 6.85, 0.0, 8, "upper_chest")
    myModel.root.children[2].children[0].children.append(joint)
    # right shoulder
    joint = Joint(-0.645, 7.125, 0.0, -1, "shoulder_right")
    myModel.root.children[2].children[0].children[0].children.append(joint)
    # right upper arm
    joint = Joint(-0.873, 7.125, 0.0, 9, "upper_arm_right")
    myModel.root.children[2].children[0].children[0].children[0].children.append(joint)
    # right lower arm
    joint = Joint(-2.325, 7.125, 0.0, 10, "lower_arm_right")
    myModel.root.children[2].children[0].children[0].children[0].children[0].children.append(joint)
    # right hand
    joint = Joint(-3.625, 7.125, 0.0, -1, "hand_right")
    myModel.root.children[2].children[0].children[0].children[0].children[0].children[0].children.append(joint)
    # left shoulder
    joint = Joint(0.645, 7.125, 0.0, -1, "shoulder_left")
    myModel.root.children[2].children[0].children[0].children.append(joint)
    # left upper arm
    joint = Joint(0.873, 7.125, 0.0, 11, "upper_arm_left")
    myModel.root.children[2].children[0].children[0].children[1].children.append(joint)
    # left lower arm
    joint = Joint(2.325, 7.125, 0.0, 12, "lower_arm_left")
    myModel.root.children[2].children[0].children[0].children[1].children[0].children.append(joint)
    # left hand
    joint = Joint(3.625, 7.125, 0.0, -1, "hand_left")
    myModel.root.children[2].children[0].children[0].children[1].children[0].children[0].children.append(joint)
    # neck
    joint = Joint(0.0, 7.4, 0.0, -1, "neck")
    myModel.root.children[2].children[0].children[0].children.append(joint)
    # head
    joint = Joint(0.0, 8.12, 0.0, -1, "head")
    myModel.root.children[2].children[0].children[0].children[2].children.append(joint)
    # right eye
    joint = Joint(-0.24, 8.25, 0.45, -1, "eye_right")
    myModel.root.children[2].children[0].children[0].children[2].children[0].children.append(joint)
    # left eye
    joint = Joint(0.24, 8.25, 0.45, -1, "eye_left")
    myModel.root.children[2].children[0].children[0].children[2].children[0].children.append(joint)
    # jaw
    joint = Joint(0.0, 7.595, 0.27, -1, "jaw")
    myModel.root.children[2].children[0].children[0].children[2].children[0].children.append(joint)


def read_data(filename1, filename2):
    """
    This function reads input files and stores the data into corresponding data structures.

    :param filename1: input filename (front view)
    :param filename2: input filename (side view)
    """

    global frames, factor, data

    # read animation info and joints' positions
    try:
        with open(filename1) as file1, open(filename2) as file2:
            line1 = file1.readline()
            line1 = line1.strip()
            splitLine1 = line1.split()
            frames = int(splitLine1[1])
            line1 = file1.readline()
            line1 = line1.strip()
            splitLine1 = line1.split()
            factor = float(splitLine1[1])
            line1 = file1.readline()
            line1 = line1.strip()
            splitLine1 = line1.split()
            joints = len(splitLine1)
            for _ in range(frames):
                data.append(list())
                line1 = file1.readline()
                line1 = line1.strip()
                splitLine1 = line1.split()
                line2 = file2.readline()
                line2 = line2.strip()
                splitLine2 = line2.split()
                for i in range(joints):
                    data[-1].append([float(splitLine1[i * 2]), -float(splitLine1[i * 2 + 1]), -float(splitLine2[i])])
    except:
        print(os.getcwd() + filename1 + " or " + filename2 + " may not exist.\nPlease check and try again.")


def process():
    """
    This function handles the undetected joints in the raw data (use data from adjacent frames)
    and sets initial hips' position to (0.0, 6.85, 0.0).
    """

    # handle undetected joints' xy positions
    for i in range(frames):
        for j in range(len(data[i])):
            if data[i][j][0] == 0.0 and data[i][j][1] == 0.0:
                if i == 0:
                    for k in range(i + 1, frames, 1):
                        if data[k][j][0] != 0.0 and data[k][j][1] != 0.0:
                            data[i][j][0] = data[k][j][0]
                            data[i][j][1] = data[k][j][1]
                            if data[i][j][2] != 0.0 and data[k][j][2] != 0.0:
                                data[i][j][2] = (data[i][j][2] + data[k][j][2]) / 2.0
                            elif data[i][j][2] == 0.0 and data[k][j][2] != 0.0:
                                data[i][j][2] = data[k][j][2]
                            break
                else:
                    data[i][j][0] = data[i - 1][j][0]
                    data[i][j][1] = data[i - 1][j][1]
                    if data[i][j][2] != 0.0 and data[i - 1][j][2] != 0.0:
                        data[i][j][2] = (data[i][j][2] + data[i - 1][j][2]) / 2.0
                    elif data[i][j][2] == 0.0 and data[i - 1][j][2] != 0.0:
                        data[i][j][2] = data[i - 1][j][2]

    # handle undetected joints' z positions
    for i in range(frames):
        for j in range(len(data[i])):
            if data[i][j][2] == 0.0:
                if i == 0:
                    for k in range(i + 1, frames, 1):
                        if data[k][j][2] != 0.0:
                            data[i][j][2] = data[k][j][2]
                            break
                else:
                    data[i][j][2] = data[i - 1][j][2]

    # set hips' initial position to (0.0, 0.0, 0.0)
    temp1 = data[0][8][0]
    temp2 = data[0][8][1]
    temp3 = data[0][8][2]

    for i in range(frames):
        for j in range(len(data[i])):
            data[i][j][0] -= data[0][8][0]
            data[i][j][0] /= factor
            data[i][j][1] -= data[0][8][1]
            data[i][j][1] /= factor
            data[i][j][1] += 6.85
            data[i][j][2] -= data[0][8][2]
            data[i][j][2] /= factor


def calculate_vector():
    """
    This function converts the raw data (positions of joints) to vectors for motion data (rotations and translations).
    """

    global vectors

    # auxiliary variables for holding the values of x, y, and z components
    temp1 = None
    temp2 = None
    temp3 = None

    for i in range(frames):
        vectors.append(list())

        # 00 hips position
        vectors[-1].append([data[i][0][0], data[i][0][1], data[i][0][2]])

        # 01 right upper leg -> left upper leg
        temp1 = data[i][9]
        temp2 = data[i][12]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 02 right upper leg -> right lower leg
        temp1 = data[i][9]
        temp2 = data[i][10]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 03 right lower leg -> right heel
        temp1 = data[i][10]
        temp2 = data[i][24]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 04 right heel -> right toes
        temp1 = data[i][24]
        temp2 = data[i][22]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 05 left upper leg -> left lower leg
        temp1 = data[i][12]
        temp2 = data[i][13]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 06 left lower leg -> left heel
        temp1 = data[i][13]
        temp2 = data[i][21]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 07 left heel -> left toes
        temp1 = data[i][21]
        temp2 = data[i][19]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 08 right upper arm -> left upper arm
        temp1 = data[i][2]
        temp2 = data[i][5]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 09 right upper arm -> right lower arm
        temp1 = data[i][2]
        temp2 = data[i][3]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 10 right lower arm -> right hand
        temp1 = data[i][3]
        temp2 = data[i][4]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 11 left upper arm -> left lower arm
        temp1 = data[i][5]
        temp2 = data[i][6]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)

        # 12 left lower arm -> left hand
        temp1 = data[i][6]
        temp2 = data[i][7]
        temp3 = [temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]]
        normalize(temp3)
        vectors[-1].append(temp3)


def keyframing():
    """
    The function performs linear keyframing.
    """
    global interval

    interval = 5

    # auxiliary variables to hold vectors for calculating rotations (quaternions)
    temp1 = None
    temp2 = None
    temp3 = None

    for i in range(len(vectors)):
        if i % interval != 0:
            for j in range(len(vectors[0])):
                temp1 = vectors[(i // interval) * interval][j]
                temp2 = vectors[((i // interval) + 1) * interval][j]
                temp3 = list()
                if j != 0:
                    dot = dotProduct(temp1, temp2)
                    if dot < 1 and dot > -1:
                        theta = math.acos(dot)
                        axis = crossProduct(temp1, temp2)
                        qTemp = Quaternion(0.0, temp1[0], temp1[1], temp1[2])
                        qRotate = Quaternion(math.cos(theta * (i % interval) / interval / 2),
                                             math.sin(theta * (i % interval) / interval / 2) * axis[0],
                                             math.sin(theta * (i % interval) / interval / 2) * axis[1],
                                             math.sin(theta * (i % interval) / interval / 2) * axis[2])
                        qTemp = qRotate.product(qTemp).product(qRotate.inverse())
                        temp3.append(qTemp.x)
                        temp3.append(qTemp.y)
                        temp3.append(qTemp.z)
                    else:
                        temp3.append(vectors[i // interval][j][0])
                        temp3.append(vectors[i // interval][j][1])
                        temp3.append(vectors[i // interval][j][2])
                else:
                    temp3.append(temp1[0] + (temp2[0] - temp1[0]) * (i % interval) / interval)
                    temp3.append(temp1[1] + (temp2[1] - temp1[1]) * (i % interval) / interval)
                    temp3.append(temp1[2] + (temp2[2] - temp1[2]) * (i % interval) / interval)
                vectors[i][j][0] = temp3[0]
                vectors[i][j][1] = temp3[1]
                vectors[i][j][2] = temp3[2]


def normalize(aList):
    """
    This function normalizes the given vector.

    :param aList: a given vector
    """

    if isinstance(aList, list):
        temp = math.sqrt(pow(aList[0], 2) + pow(aList[1], 2) + pow(aList[2], 2))
        aList[0] /= temp
        aList[1] /= temp
        aList[2] /= temp
    else:
        print("The argument of .normalize() function should be a List...")
        sys.exit(-1)


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

    with open(filename, 'w') as file:
        file.write("# import module that contains most Maya MEL commands to Python\n")
        file.write("import maya.cmds as cmds\n\n")
        file.write("# delete keysframes of all joints\n")
        for joint in joints:
            file.write("cmds.cutKey('" + joint + "', time=(1, " + str(frames) + "))\n")
        file.write("\n# set keyframes\n")
        for i in range(len(vectors)):
            reset(myModel.root)
            transformation(myModel.root, i, file)


def reset(current):
    """
    This function resets the model.

    :param current: current joint to be reset
    """
    current.x = current.ox
    current.y = current.oy
    current.z = current.oz
    current.rotate = None
    for joint in current.children:
        reset(joint)


def transformation(current, i, file):
    """
    This function performs transformation of a joint according to the current frame.

    :param current: the joint to be transformed now
    :param i: current frame
    :param file: file to write in
    """

    if current.index != -1:
        # the current vector from this joint to one of its children
        vector = None
        # the vector the current vector should be
        vectorToBe = None
        # dot production
        dot = None
        # rotation angle
        theta = None
        # rotation axis
        axis = None
        # rotation quaternion
        qRotate = None

        # hips (needs traslation and rotation)
        if current.index == 0:
            translation(current, vectors[i][0][0] - current.x, vectors[i][0][1] - current.y,
                        vectors[i][0][2] - current.z, file, True, i + 1)

            vector = list()
            vector.append(current.children[1].x - current.children[0].x)
            vector.append(current.children[1].y - current.children[0].y)
            vector.append(current.children[1].z - current.children[0].z)
            normalize(vector)
            vectorToBe = vectors[i][1]
            dot = dotProduct(vector, vectorToBe)
            if dot < 1 and dot > -1:
                theta = math.acos(dot)
                axis = crossProduct(vector, vectorToBe)
                qRotate = Quaternion(math.cos(theta / 2), math.sin(theta / 2) * axis[0], math.sin(theta / 2) * axis[1],
                                     math.sin(theta / 2) * axis[2])
                rotation(current, current.x, current.y, current.z, qRotate, file, True, i + 1)

        # upper chest
        elif current.index == 8:
            vector = list()
            vector.append(current.children[1].x - current.children[0].x)
            vector.append(current.children[1].y - current.children[0].y)
            vector.append(current.children[1].z - current.children[0].z)
            normalize(vector)
            vectorToBe = vectors[i][current.index]
            dot = dotProduct(vector, vectorToBe)
            if dot < 1 and dot > -1:
                theta = math.acos(dot)
                axis = crossProduct(vector, vectorToBe)
                qRotate = Quaternion(math.cos(theta / 2), math.sin(theta / 2) * axis[0], math.sin(theta / 2) * axis[1],
                                     math.sin(theta / 2) * axis[2])
                rotation(current, current.x, current.y, current.z, qRotate, file, True, i + 1)

        # other joints
        else:
            vector = list()
            vector.append(current.children[0].x - current.x)
            vector.append(current.children[0].y - current.y)
            vector.append(current.children[0].z - current.z)
            normalize(vector)
            vectorToBe = vectors[i][current.index]
            dot = dotProduct(vector, vectorToBe)
            if dot < 1 and dot > -1:
                theta = math.acos(dot)
                axis = crossProduct(vector, vectorToBe)
                qRotate = Quaternion(math.cos(theta / 2), math.sin(theta / 2) * axis[0], math.sin(theta / 2) * axis[1],
                                     math.sin(theta / 2) * axis[2])
                rotation(current, current.x, current.y, current.z, qRotate, file, True, i + 1)

    # check transformations of the current joint's children
    for joint in current.children:
        transformation(joint, i, file)


def translation(current, dx, dy, dz, file, toWrite, time):
    """
    This is a helper function of transformation that handles translation.

    :param current: the current joint
    :param dx: x translation
    :param dy: y translation
    :param dz: z translation
    :param file: file to write in
    :param toWrite: to write or not
    :param time: current keyframe
    """
    current.x += dx
    current.y += dy
    current.z += dz
    if toWrite:
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='translateX', value=" + str(round(dx, 3)) + ")\n")
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='translateY', value=" + str(round(dy, 3)) + ")\n")
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='translateZ', value=" + str(round(dz, 3)) + ")\n")
    for joint in current.children:
        translation(joint, dx, dy, dz, file, False, time)


def rotation(current, dx, dy, dz, qRotate, file, toWrite, time):
    """
    This is a helper function of transformation that handles rotation.

    :param current: the current joint
    :param dx: x position of the joint at the rotation center
    :param dy: y position of the joint at the rotation center
    :param dz: z position of the joint at the rotation center
    :param qRotate: the rotation quaternion
    :param file: file to write in
    :param toWrite: to write or not
    :param time: current keyframe
    """

    qTemp = None
    qq = None
    if current.rotate is not None:
        qq = current.rotate.inverse().product(qRotate).product(current.rotate)
    else:
        qq = qRotate
    if toWrite:
        angles = quaterenion_to_euler(qq)
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='rotateX', value=" + str(round(angles[0], 3)) + ")\n")
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='rotateY', value=" + str(round(angles[1], 3)) + ")\n")
        file.write("cmds.setKeyframe('" + current.name + "', time=" + str(time) +
                   ", attribute='rotateZ', value=" + str(round(angles[2], 3)) + ")\n")

    for joint in current.children:
        if joint.rotate is None:
            joint.rotate = qq
        else:
            joint.rotate = joint.rotate.product(qq)
        joint.x -= dx
        joint.y -= dy
        joint.z -= dz
        qTemp = Quaternion(0.0, joint.x, joint.y, joint.z)
        qTemp = qRotate.product(qTemp).product(qRotate.inverse())
        joint.x = qTemp.x
        joint.y = qTemp.y
        joint.z = qTemp.z
        joint.x += dx
        joint.y += dy
        joint.z += dz

    # inherited rotation
    for joint in current.children:
        rotation(joint, dx, dy, dz, qRotate, file, False, time)


def quaterenion_to_euler(q):
    """
    This function converts a quaternion to its corresponding Euler's angles.

    :param q: the quaternion to be converted
    :return: corresponding Euler's angles
    """
    temp = list()

    # rotX angle
    sinr_cosp = 2 * (q.w * q.x + q.y * q.z)
    cosr_cosp = 1 - 2 * (q.x * q.x + q.y * q.y)
    temp.append((180.0 / math.pi) * math.atan2(sinr_cosp, cosr_cosp))

    # rotY angle
    sinp = 2 * (q.w * q.y - q.x * q.z)
    if sinp > 1 or sinp < -1:
        temp.append((180.0 / math.pi) * math.copysign(math.pi / 2, sinp))
    else:
        temp.append((180.0 / math.pi) * math.asin(sinp))

    # rotZ angle
    siny_cosp = 2 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
    temp.append((180.0 / math.pi) * math.atan2(siny_cosp, cosy_cosp))
    return temp


def dotProduct(v1, v2):
    """
    This function performs dot production of two vectors and returns the result.

    :param v1: the first vector
    :param v2: the second vector
    :return: the dot production of the input vectors
    """
    return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2]


def crossProduct(v1, v2):
    """
    This function performs cross production of two vectors and returns the result.

    :param v1: the first vector
    :param v2: the second vector
    :return: the cross production of the input vectors
    """

    temp = list()
    temp.append(v1[1] * v2[2] - v2[1] * v1[2])
    temp.append(v1[2] * v2[0] - v2[2] * v1[0])
    temp.append(v1[0] * v2[1] - v2[0] * v1[1])
    normalize(temp)
    return temp


'''
main conditional guard
The following condition checks whether we are running as a script.
If the file is being imported, don't run the test code.
'''
if __name__ == '__main__':
    if len(sys.argv) != 4:
        print("Usage: cv_redaer.py input_file_1_pathname(front) input_file_2_pathname(side) output_filename")
        sys.exit(-1)
    else:
        set_model()
        read_data(sys.argv[1], sys.argv[2])
        process()
        calculate_vector()
        keyframing()
        write_data(sys.argv[3])
