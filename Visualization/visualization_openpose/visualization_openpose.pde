/**
 * File: visualization_openpose.pde
 * This is the main code for visualization of CV-based mocap from capstone project, Motion Capture Pipeline for Virtual World.
 *
 * @author Michael Lee, ml3406@rit.edu
 */

/** directory of the input data */
String directory = "I:/RIT/2022 Fall/visualization_openpose/data/";

/** which motion to read (Boxing/Moonwalk/Wave) */
String motion = "Wave";

/** to apply keyframing or not */
boolean keyframe = true;

/** frame rate */
float fps = 30.0;

/** a simple model (skeleton with joints and bones only) */
Model myModel;

/** all joints of the model */
Joint hips, rightUpperLeg, rightLowerLeg, rightFoot, rightToes, leftUpperLeg, leftLowerLeg, leftFoot, leftToes, spine, chest, upperChest, rightShoulder, rightUpperArm, rightLowerArm, rightHand, leftShoulder, leftUpperArm, leftLowerArm, leftHand, neck, head, rightEye, leftEye, jaw;

/** a factor that scales the model */
float factor1 = 39.0;

/** a factor that handles scaling of traslations in motion data */
float factor2;

/** current frame */
int currentFrame;

/** total number of frames */
int frames;

/** keyframe interval */
int interval = 5;

/** a list to store raw data (joints' positions at every frame) */
ArrayList<ArrayList<float[]>> data;

/** a list to store motion data */
ArrayList<ArrayList<float[]>> vectors;

/**
 * This function is excuted once at the beginning to initialze all important fields.
 */
void setup() {
  // canvas size 1760 * 990 (3D space)
  size(1760, 990, P3D);

  // camera position (+x: right; +y: down; +z: out)
  camera(0.0, 0.0, 550.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  //camera(450.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

  // set up a model for visualization
  setModel();

  // read data
  input();

  // handle undetected joints and set initial hips' position to (0.0, 0.0, 0.0)
  process();

  // convert raw data (joints' positions) to vectors (for calculating rotations)
  calculate_vector();

  // keyframing - linear interpolation
  if (keyframe) keyframing();

  // set up frame rate
  frameRate(fps);

  // current frame is 0 initially
  currentFrame = 0;

  // subdivision of spheres
  sphereDetail(6);
}

/**
 * This function is executed repeatedly for producing animation
 */
void draw() {
  // animation is not over
  if (currentFrame < vectors.size()) {
    background(0);
    // transform the model based on the motion data
    transformation(myModel.root);
    // draw the model
    drawModel(myModel.root);
    currentFrame++;
  }
  // buffer time before replaying the animation
  else if (currentFrame < vectors.size() + 3) {
    background(0);
    currentFrame++;
  }
  // reset the model and current frame time to replay the animation
  else {
    setModel();
    currentFrame = 0;
  }
}

/**
 * This function draws the model on the canvas.
 *
 * @param current - the joint to be drawn now
 */
void drawModel(Joint current) {
  // the joint
  pushMatrix();
  translate(factor1 * current.x, -factor1 * current.y, factor1 * current.z);
  stroke(200, 200, 0);
  sphere(3);
  popMatrix();

  // bones coonected to the current joint
  for (Joint j : current.children) {
    stroke(255);
    line(factor1 * current.x, -factor1 * current.y, factor1 * current.z, factor1 * j.x, -factor1 * j.y, factor1 * j.z);
    drawModel(j);
  }
}

/**
 *  This function reads input files and stores the data into corresponding data structures.
 */
void input() {
  String[] lines = loadStrings(directory + motion + "-front.txt");
  String[] splitLine;

  int index = 0;
  frames = Integer.parseInt(lines[index++].split("\\s+")[1]);
  factor2 = Float.parseFloat(lines[index++].split("\\s+")[1]);
  
  // total number of joints
  int joints = lines[index++].split("\\s+").length;

  // xy positions (from the front view)
  data = new ArrayList();
  for (int i = 0; i < frames; i++) {
    data.add(new ArrayList());
    splitLine = lines[index++].split("\\s+");
    for (int j = 0; j < joints; j++) {
      data.get(data.size() - 1).add(new float[3]);
      data.get(i).get(j)[0] = Float.parseFloat(splitLine[j * 2]);
      data.get(i).get(j)[1] = -Float.parseFloat(splitLine[j * 2 + 1]);
    }
  }

  // z positions (from the right-side view)
  lines = loadStrings(directory + motion + "-side.txt");
  index = 0;
  splitLine = lines[index].split("\\s+");

  for (int i = 0; i < frames; i++) {
    splitLine = lines[index++].split("\\s+");
    for (int j = 0; j < joints; j++) {
      data.get(i).get(j)[2] = -Float.parseFloat(splitLine[j]);
    }
  }
}

/**
 * This function handles the undetected joints in the raw data (use data from adjacent frames)
 * and set initial hips' position to (0.0, 0.0, 0.0).
 */
void process() {
  // handle undetected joints
  // xy positions
  for (int i = 0; i < frames; i++) {
    for (int j = 0; j < data.get(i).size(); j++) {
      if (data.get(i).get(j)[0] == 0.0 && data.get(i).get(j)[1] == 0.0) {
        if (i == 0) {
          for (int k = i + 1; k < frames; k++) {
            if (data.get(k).get(j)[0] != 0.0 && data.get(k).get(j)[1] != 0.0) {
              data.get(i).get(j)[0] = data.get(k).get(j)[0];
              data.get(i).get(j)[1] = data.get(k).get(j)[1];
              if (data.get(i).get(j)[2] != 0.0 && data.get(k).get(j)[2] != 0.0) data.get(i).get(j)[2] = (data.get(i).get(j)[2] + data.get(k).get(j)[2]) / 2.0;
              else if (data.get(i).get(j)[2] == 0.0 && data.get(k).get(j)[2] != 0.0) data.get(i).get(j)[2] = data.get(k).get(j)[2];
              break;
            }
          }
        } else {
          data.get(i).get(j)[0] = data.get(i - 1).get(j)[0];
          data.get(i).get(j)[1] = data.get(i - 1).get(j)[1];
          if (data.get(i).get(j)[2] != 0.0 && data.get(i - 1).get(j)[2] != 0.0) data.get(i).get(j)[2] = (data.get(i).get(j)[2] + data.get(i - 1).get(j)[2]) / 2.0;
          else if (data.get(i).get(j)[2] == 0.0 && data.get(i - 1).get(j)[2] != 0.0) data.get(i).get(j)[2] = data.get(i - 1).get(j)[2];
        }
      }
    }
  }
  // z positions
  for (int i = 0; i < frames; i++) {
    for (int j = 0; j < data.get(i).size(); j++) {
      if (data.get(i).get(j)[2] == 0.0) {
        if (i == 0) {
          for (int k = i + 1; k < frames; k++) {
            if (data.get(k).get(j)[2] != 0.0) {
              data.get(i).get(j)[2] = data.get(k).get(j)[2];
              break;
            }
          }
        } else {
          data.get(i).get(j)[2] = data.get(i - 1).get(j)[2];
        }
      }
    }
  }

  // set hips' initial position to (0.0, 0.0, 0.0)
  float f1 = data.get(0).get(8)[0];
  float f2 = data.get(0).get(8)[1];
  float f3 = data.get(0).get(8)[2];
  for (int i = 0; i < frames; i++) {
    for (int j = 0; j < data.get(i).size(); j++) {
      data.get(i).get(j)[0] -= f1;
      data.get(i).get(j)[0] /= factor2;
      data.get(i).get(j)[1] -= f2;
      data.get(i).get(j)[1] /= factor2;
      data.get(i).get(j)[2] -= f3;
      data.get(i).get(j)[2] /= factor2;
    }
  }
}

/**
 * This function converts the raw data (positions of joints) to vectors for motion data (rotations and translations).
 */
void calculate_vector() {
  // auxiliary variables for holding the values of x, y, and z components
  float[] temp1, temp2, temp3;

  vectors = new ArrayList();
  for (int i = 0; i < frames; i++) {
    vectors.add(new ArrayList());

    // hips position
    vectors.get(vectors.size() - 1).add(data.get(i).get(8));

    // hips -> upper chest
    temp1 = data.get(i).get(1);
    temp2 = data.get(i).get(8);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right upper leg -> left upper leg
    temp1 = data.get(i).get(9);
    temp2 = data.get(i).get(12);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right upper leg -> right lower leg
    temp1 = data.get(i).get(9);
    temp2 = data.get(i).get(10);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right lower leg -> right heel
    temp1 = data.get(i).get(10);
    temp2 = data.get(i).get(24);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right heel -> right toes
    temp1 = data.get(i).get(24);
    temp2 = data.get(i).get(22);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // left upper leg -> left lower leg
    temp1 = data.get(i).get(12);
    temp2 = data.get(i).get(13);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // left lower leg -> left heel
    temp1 = data.get(i).get(13);
    temp2 = data.get(i).get(21);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // left heel -> left toes
    temp1 = data.get(i).get(21);
    temp2 = data.get(i).get(19);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // upper chest -> head
    temp1 = data.get(i).get(1);
    temp2 = data.get(i).get(0);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right upper arm -> left upper arm
    temp1 = data.get(i).get(2);
    temp2 = data.get(i).get(5);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right upper arm -> right lower arm
    temp1 = data.get(i).get(2);
    temp2 = data.get(i).get(3);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // right lower arm -> right hand
    temp1 = data.get(i).get(3);
    temp2 = data.get(i).get(4);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // left upper arm -> left lower arm
    temp1 = data.get(i).get(5);
    temp2 = data.get(i).get(6);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);

    // left lower arm -> left hand
    temp1 = data.get(i).get(6);
    temp2 = data.get(i).get(7);
    temp3 = new float[]{temp2[0] - temp1[0], temp2[1] - temp1[1], temp2[2] - temp1[2]};
    normalize(temp3);
    vectors.get(vectors.size() - 1).add(temp3);
  }
}

/**
 * The function performs linear keyframing with interval of 5.
 */
void keyframing() {
  // auxiliary variables to hold vectors for calculating rotations (quaternions)
  float[] temp1, temp2, temp3;

  // rotation axis
  float[] axis;

  // dot production and rotation angle
  float dot, theta;

  // current position quaternion and rotation quaternion
  Quaternion qP, qRotate;

  for (int i = 0; i < vectors.size(); i++) {
    if (i % interval != 0) {
      for (int j = 0; j < vectors.get(0).size(); j++) {
        temp1 = vectors.get((i / interval) * interval).get(j);
        temp2 = vectors.get((i / interval + 1) * interval).get(j);
        temp3 = new float[3];
        if (j != 0) {
          dot = dotProduct(temp1, temp2);
          if (dot < 1 && dot > -1) {
            theta = acos(dot);
            axis = crossProduct(temp1, temp2);
            qP = new Quaternion(0.0, temp1[0], temp1[1], temp1[2]);
            qRotate = new Quaternion(cos(theta * (i % interval) / 5 / 2), sin(theta * (i % interval) / 5 / 2) * axis[0], sin(theta * (i % interval) / 5 / 2) * axis[1], sin(theta * (i % interval) / 5 / 2) * axis[2]);
            qP = qRotate.product(qP).product(qRotate.inverse());
            temp3[0] = qP.x;
            temp3[1] = qP.y;
            temp3[2] = qP.z;
          } else {
            temp3[0] = vectors.get(i / interval).get(j)[0];
            temp3[1] = vectors.get(i / interval).get(j)[1];
            temp3[2] = vectors.get(i / interval).get(j)[2];
          }
        } else {
          temp3[0] = temp1[0] + (temp2[0] - temp1[0]) * (i % interval) / 5;
          temp3[1] = temp1[1] + (temp2[1] - temp1[1]) * (i % interval) / 5;
          temp3[2] = temp1[2] + (temp2[2] - temp1[2]) * (i % interval) / 5;
        }
        vectors.get(i).get(j)[0] = temp3[0];
        vectors.get(i).get(j)[1] = temp3[1];
        vectors.get(i).get(j)[2] = temp3[2];
      }
    }
  }
}

/**
 * This function normalizes the given vector.
 *
 * @param vector - a given vector
 */
void normalize(float[] vector) {
  float temp = sqrt(vector[0] * vector[0] + vector[1] * vector[1] + vector[2] * vector[2]);
  vector[0] /= temp;
  vector[1] /= temp;
  vector[2] /= temp;
}

/**
 * This function performs dot production of two vectors and returns the result.
 *
 * @param v1 - the first vector
 * @param v2 - the second vector
 * @return the dot production of the input vectors
 */
float dotProduct(float[] v1, float[] v2) {
  return v1[0] * v2[0] + v1[1] * v2[1] + v1[2] * v2[2];
}

/**
 * This function performs cross production of two vectors and returns the result.
 *
 * @param v1 - the first vector
 * @param v2 - the second vector
 * @return the cross production of the input vectors
 */
float[] crossProduct(float[] v1, float[] v2) {
  float[] temp = new float[3];
  temp[0] = v1[1] * v2[2] - v2[1] * v1[2];
  temp[1] = v1[2] * v2[0] - v2[2] * v1[0];
  temp[2] = v1[0] * v2[1] - v2[0] * v1[1];
  normalize(temp);
  return temp;
}

/**
 * This function performs transformation of a joint according to the current frame.
 *
 * @param current - the joint to be transformed now
 */
void transformation(Joint current) {
  if (current.index != -1) {
    // rotation quaternion
    Quaternion qRotate;

    // the current vector from this joint to one of its children
    float[] vector = new float[3];

    // the vector the current vector should be
    float[] vectorToBe;

    // rotation axis
    float[] axis;

    // dot prodution and rotation angle
    float dot, theta;

    // hips (needs traslation and rotation)
    if (current.index == 0) {
      translation(current, vectors.get(currentFrame).get(current.index)[0] - current.x, vectors.get(currentFrame).get(current.index)[1] - current.y, vectors.get(currentFrame).get(current.index)[2] - current.z);

      // this rotation is skipped for results
      vector[0] = current.children.get(2).x - current.x;
      vector[1] = current.children.get(2).y - current.y;
      vector[2] = current.children.get(2).z - current.z;
      normalize(vector);
      vectorToBe = vectors.get(currentFrame).get(current.index + 1);
      dot = dotProduct(vector, vectorToBe);
      if (dot < 1 && dot >= -1) {
        theta = acos(dotProduct(vector, vectorToBe));
        axis = crossProduct(vector, vectorToBe);
        qRotate = new Quaternion(cos(theta / 2), sin(theta / 2) * axis[0], sin(theta / 2) * axis[1], sin(theta / 2) * axis[2]);
        //rotation(current, current.x, current.y, current.z, qRotate);
      }

      vector[0] = current.children.get(1).x - current.children.get(0).x;
      vector[1] = current.children.get(1).y - current.children.get(0).y;
      vector[2] = current.children.get(1).z - current.children.get(0).z;
      normalize(vector);
      vectorToBe = vectors.get(currentFrame).get(current.index + 2);
      dot = dotProduct(vector, vectorToBe);
      if (dot < 1 && dot >= -1) {
        theta = acos(dotProduct(vector, vectorToBe));
        axis = crossProduct(vector, vectorToBe);
        qRotate = new Quaternion(cos(theta / 2), sin(theta / 2) * axis[0], sin(theta / 2) * axis[1], sin(theta / 2) * axis[2]);
        rotation(current, current.x, current.y, current.z, qRotate);
      }
    }
    // upper chest
    else if (current.index == 9) {
      // this rotation is skipped for results
      vector[0] = current.children.get(2).x - current.x;
      vector[1] = current.children.get(2).y - current.y;
      vector[2] = current.children.get(2).z - current.z;
      normalize(vector);
      vectorToBe = vectors.get(currentFrame).get(current.index);
      dot = dotProduct(vector, vectorToBe);
      if (dot < 1 && dot >= -1) {
        theta = acos(dotProduct(vector, vectorToBe));
        axis = crossProduct(vector, vectorToBe);
        qRotate = new Quaternion(cos(theta / 2), sin(theta / 2) * axis[0], sin(theta / 2) * axis[1], sin(theta / 2) * axis[2]);
        //rotation(current, current.x, current.y, current.z, qRotate);
      }

      vector[0] = current.children.get(1).x - current.children.get(0).x;
      vector[1] = current.children.get(1).y - current.children.get(0).y;
      vector[2] = current.children.get(1).z - current.children.get(0).z;
      normalize(vector);
      vectorToBe = vectors.get(currentFrame).get(current.index + 1);
      dot = dotProduct(vector, vectorToBe);
      if (dot < 1 && dot >= -1) {
        theta = acos(dotProduct(vector, vectorToBe));
        axis = crossProduct(vector, vectorToBe);
        qRotate = new Quaternion(cos(theta / 2), sin(theta / 2) * axis[0], sin(theta / 2) * axis[1], sin(theta / 2) * axis[2]);
        rotation(current, current.x, current.y, current.z, qRotate);
      }
    } 
    // other joints
    else {
      vector[0] = current.children.get(0).x - current.x;
      vector[1] = current.children.get(0).y - current.y;
      vector[2] = current.children.get(0).z - current.z;
      normalize(vector);
      vectorToBe = vectors.get(currentFrame).get(current.index);
      dot = dotProduct(vector, vectorToBe);
      if (dot < 1 && dot >= -1) {
        theta = acos(dotProduct(vector, vectorToBe));
        axis = crossProduct(vector, vectorToBe);
        qRotate = new Quaternion(cos(theta / 2), sin(theta / 2) * axis[0], sin(theta / 2) * axis[1], sin(theta / 2) * axis[2]);
        rotation(current, current.x, current.y, current.z, qRotate);
      }
    }
  }

  // check transformations of the current joint's children
  for (Joint j : current.children) {
    transformation(j);
  }
}

/**
 * This is a helper function of transformation that handles translation.
 *
 * @param current - the current joint
 * @param dx      - x translation
 * @param dy      - y translation
 * @param dz      - z translation
 */
void translation(Joint current, float dx, float dy, float dz) {
  current.x += dx;
  current.y += dy;
  current.z += dz;
  for (Joint j : current.children) {
    translation(j, dx, dy, dz);
  }
}

/**
 * This is a helper function of transformation that handles rotation.
 *
 * @param current - the current joint
 * @param dx      - x position of the joint at the rotation center
 * @param dy      - y position of the joint at the rotation center
 * @param dz      - z position of the joint at the rotation center
 * @param rotateQ - the rotation quaternion
 */
void rotation(Joint current, float dx, float dy, float dz, Quaternion qRotate) {
  for (Joint j : current.children) {
    j.x -= dx;
    j.y -= dy;
    j.z -= dz;
    Quaternion qTemp = new Quaternion(0.0, j.x, j.y, j.z);
    qTemp = qRotate.product(qTemp).product(qRotate.inverse());
    j.x = qTemp.x;
    j.y = qTemp.y;
    j.z = qTemp.z;
    j.x += dx;
    j.y += dy;
    j.z += dz;
  }
  
  // inherited rotation
  for (Joint j : current.children) {
    rotation(j, dx, dy, dz, qRotate);
  }
}

/**
 * This function sets up a model in T-pose.
 */
void setModel() {
  myModel = new Model();
  // hips
  hips = new Joint(0.0, 4.6, 0.0, 0);
  myModel.root = hips;
  // right upper leg
  rightUpperLeg = new Joint(-0.45, 4.28, 0.0, 3);
  myModel.root.children.add(rightUpperLeg);
  // right lower leg
  rightLowerLeg = new Joint(-0.45, 2.265, 0.0, 4);
  myModel.root.children.get(0).children.add(rightLowerLeg);
  // right foot
  rightFoot = new Joint(-0.45, 0.15, 0.0, 5);
  myModel.root.children.get(0).children.get(0).children.add(rightFoot);
  // right toes
  rightToes = new Joint(-0.45, 0.15, 1.0, -1);
  myModel.root.children.get(0).children.get(0).children.get(0).children.add(rightToes);
  // left upper leg
  leftUpperLeg = new Joint(0.45, 4.28, 0.0, 6);
  myModel.root.children.add(leftUpperLeg);
  // left lower leg
  leftLowerLeg = new Joint(0.45, 2.265, 0.0, 7);
  myModel.root.children.get(1).children.add(leftLowerLeg);
  // left foot
  leftFoot = new Joint(0.45, 0.15, 0.0, 8);
  myModel.root.children.get(1).children.get(0).children.add(leftFoot);
  // left toes
  leftToes = new Joint(0.45, 0.15, 1.0, -1);
  myModel.root.children.get(1).children.get(0).children.get(0).children.add(leftToes);
  // spine
  spine = new Joint(0.0, 5.3, 0.0, -1);
  myModel.root.children.add(spine);
  // chest
  chest = new Joint(0.0, 6.05, 0.0, -1);
  myModel.root.children.get(2).children.add(chest);
  // upper chest
  upperChest = new Joint(0.0, 6.85, 0.0, 9);
  myModel.root.children.get(2).children.get(0).children.add(upperChest);
  // right shoulder
  rightShoulder = new Joint(-0.645, 7.125, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(rightShoulder);
  // right upper arm
  rightUpperArm = new Joint(-0.873, 7.125, 0.0, 11);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.add(rightUpperArm);
  // right lower arm
  rightLowerArm = new Joint(-2.325, 7.125, 0.0, 12);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.get(0).children.add(rightLowerArm);
  // right hand
  rightHand = new Joint(-3.625, 7.125, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.get(0).children.get(0).children.add(rightHand);
  // left shoulder
  leftShoulder = new Joint(0.645, 7.125, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(leftShoulder);
  // left upper arm
  leftUpperArm = new Joint(0.873, 7.125, 0.0, 13);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.add(leftUpperArm);
  // left lower arm
  leftLowerArm = new Joint(2.325, 7.125, 0.0, 14);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.get(0).children.add(leftLowerArm);
  // left hand
  leftHand = new Joint(3.625, 7.125, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.get(0).children.get(0).children.add(leftHand);
  // neck
  neck = new Joint(0.0, 7.4, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(neck);
  // head
  head = new Joint(0.0, 8.12, 0.0, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.add(head);
  // right eye
  rightEye = new Joint(-0.24, 8.25, 0.45, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(rightEye);
  // left eye
  leftEye = new Joint(0.24, 8.25, 0.45, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(leftEye);
  // jaw
  jaw = new Joint(0.0, 7.595, 0.27, -1);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(jaw);
}
