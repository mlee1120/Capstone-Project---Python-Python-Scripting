/** //<>//
 * File: visualization_bvh.pde
 * This is the main code for visualization of mocap using database from capstone project, Motion Capture Pipeline for Virtual World.
 *
 * @author Michael Lee, ml3406@rit.edu
 */

/** directory of the input data */
String directory = "I:/RIT/2022 Fall/visualization_bvh/data/";

/** which motion to read (Ambient/Arms/Circle/Jog/Legs/Pickup/Sit/Sneak/Stand/Tiptoe/Turn/Wave) */
String motion = "Jog";

/** a simple model (skeleton with joints and bones only) */
Model myModel;

/** all joints of the model */
Joint hips, rightUpperLeg, rightLowerLeg, rightFoot, rightToes, leftUpperLeg, leftLowerLeg, leftFoot, leftToes, spine, chest, upperChest, rightShoulder, rightUpperArm, rightLowerArm, rightHand, leftShoulder, leftUpperArm, leftLowerArm, leftHand, neck, head, rightEye, leftEye, jaw;

/** a factor that scales the model */
float factor1 = 39.0;

/** a factor that handle scaling of traslation in motion data */
float factor2 = 30.0;

/** total number of frames */
float frames;

/** time duration per frame */
float frameTime;

/** current frame (used to decide which line of data to use and when to restart) */
int currentFrame;

/** a list to store all motion capture data */
ArrayList<ArrayList<Float>> data;

/**
 * This function is excuted once at the beginning to initialze all important fields.
 */
void setup() {
  // canvas size 1760 * 990 (3D space)
  size(1760, 990, P3D);

  // camera position (+x: right; +y: down; +z: out)
  camera(0.0, -factor1 * 4.125, 750.0, 0.0, -factor1 * 4.125, 0.0, 0.0, 1.0, 0.0);
  //camera(450.0, -ss * 4.125, 0.0, 0.0, -ss * 4.125, 0.0, 0.0, 1.0, 0.0);

  // set up a model for visualization
  setModel();

  // read data
  input();

  // set up frame rate according to the input data
  frameRate(1.0 / frameTime);

  // current frame is 0 initially
  currentFrame = 0;

  // subdivision of spheres
  sphereDetail(6);
}

/**
 * This function is executed repeatedly for producing animation
 */
void draw() {
  background(0);
  // transform the model from leaves to the root to avoid handling world and object spaces
  transformation(rightFoot);
  transformation(leftFoot);
  transformation(rightHand);
  transformation(leftHand);
  transformation(head);

  // draw the model
  drawModel(myModel.root);
  currentFrame++;

  // replay the animation
  if (currentFrame == frames) currentFrame = 0;

  // reset the model
  reset(myModel.root);
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
 *  This function reads .bvh file and stores the data into corresponding data structures.
 */
void input() {
  String[] lines = loadStrings(directory + motion + ".bvh");
  String[] splitLine;
  hips.index = 0;
  int dataIndex = 2;
  int index = 3;
  int end = 1;

  // hierarchy
  do {
    splitLine = lines[index].split("\\s+");
    if (splitLine[0].equals("")) {
      switch(splitLine[1]) {
      case "OFFSET":
        break;
      case "CHANNELS":
        break;
      case "JOINT":
        if (splitLine[2].equals("LeftHip")) leftUpperLeg.index = dataIndex++;
        else if (splitLine[2].equals("LeftKnee")) leftLowerLeg.index = dataIndex++;
        else if (splitLine[2].equals("LeftAnkle")) leftFoot.index = dataIndex++;
        else if (splitLine[2].equals("RightHip")) rightUpperLeg.index = dataIndex++;
        else if (splitLine[2].equals("RightKnee")) rightLowerLeg.index = dataIndex++;
        else if (splitLine[2].equals("RightAnkle")) rightFoot.index = dataIndex++;
        else if (splitLine[2].equals("Chest")) chest.index = dataIndex++;
        else if (splitLine[2].equals("LeftCollar")) leftShoulder.index = dataIndex++;
        else if (splitLine[2].equals("LeftShoulder")) leftUpperArm.index = dataIndex++;
        else if (splitLine[2].equals("LeftElbow")) leftLowerArm.index = dataIndex++;
        else if (splitLine[2].equals("LeftWrist")) dataIndex++;
        else if (splitLine[2].equals("RightCollar")) rightShoulder.index = dataIndex++;
        else if (splitLine[2].equals("RightShoulder")) rightUpperArm.index = dataIndex++;
        else if (splitLine[2].equals("RightElbow")) rightLowerArm.index = dataIndex++;
        else if (splitLine[2].equals("RightWrist")) dataIndex++;
        else if (splitLine[2].equals("Neck")) neck.index = dataIndex++;
        else if (splitLine[2].equals("Head")) head.index = dataIndex++;
        break;
      case "End":
        break;
      case "{":
        end++;
        break;
      case "}":
        end--;
        break;
      }
    } else if (splitLine[0].equals("}")) end--;
    index++;
  } while (end != 0);

  // motion
  //skip the line "MOTION"
  index++;
  frames = Float.parseFloat(lines[index].split("\\s+")[1]);
  index++;
  frameTime = Float.parseFloat(lines[index].split("\\s+")[2]);
  index++;

  // store all motion data
  data = new ArrayList();
  for (int i = 0; i < frames; i++) {
    data.add(new ArrayList());
    splitLine = lines[index].split("\\s+");
    for (String s : splitLine) {
      data.get(data.size() - 1).add(Float.parseFloat(s));
    }
    index++;
  }
}

/**
 * This function performs transformation of a joint according to the current frame.
 *
 * @param current - the joint to be transformed now
 */
void transformation(Joint current) {
  current.toTrans = false;
  if (current.index != -1) {
    // rotation quaternions
    Quaternion Qx, Qy, Qz;

    // rotation angle
    float angle;
    if (current.index == 0) {
      translation(current, data.get(currentFrame).get(0) / factor2, data.get(currentFrame).get(1) / factor2, data.get(currentFrame).get(2) / factor2);
      angle = data.get(currentFrame).get(3);
      Qz = new Quaternion(cos(angle / 2 * PI / 180), 0.0, 0.0, sin(angle / 2 * PI / 180));
      angle = data.get(currentFrame).get(4);
      Qx = new Quaternion(cos(angle / 2 * PI / 180), sin(angle / 2 * PI / 180), 0.0, 0.0);
      angle = data.get(currentFrame).get(5);
      Qy = new Quaternion(cos(angle / 2 * PI / 180), 0.0, sin(angle / 2 * PI / 180), 0.0);
      rotation(current, current.x, current.y, current.z, Qz.product(Qx).product(Qy));
    } else {
      angle = data.get(currentFrame).get(current.index * 3);
      Qz = new Quaternion(cos(angle / 2 * PI / 180), 0.0, 0.0, sin(angle / 2 * PI / 180));
      angle = data.get(currentFrame).get(current.index * 3 + 1);
      Qx = new Quaternion(cos(angle / 2 * PI / 180), sin(angle / 2 * PI / 180), 0.0, 0.0);
      angle = data.get(currentFrame).get(current.index * 3 + 2);
      Qy = new Quaternion(cos(angle / 2 * PI / 180), 0.0, sin(angle / 2 * PI / 180), 0.0);
      rotation(current, current.x, current.y, current.z, Qz.product(Qx.product(Qy)));
    }
  }

  // to check transformation of the parent of the current joint or not
  boolean carryOn = true;
  if (current.parent == null) carryOn = false;
  else {
    for (Joint j : current.parent.children) {
      if (j.toTrans == true) {
        carryOn = false;
        break;
      }
    }
  }
  if (carryOn) {
    transformation(current.parent);
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
  current.x = current.x + dx;
  current.y = current.y + dy;
  current.z = current.z + dz;
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
 * This function sets up a model in T-pose and rotates it shouders to put its arms down.
 */
void setModel() {
  myModel = new Model();
  // hips
  hips = new Joint(0.0, 4.6, 0.0, null);
  myModel.root = hips;
  // right upper leg
  rightUpperLeg = new Joint(-0.45, 4.28, 0.0, hips);
  myModel.root.children.add(rightUpperLeg);
  // right lower leg
  rightLowerLeg = new Joint(-0.45, 2.265, 0.0, rightUpperLeg);
  myModel.root.children.get(0).children.add(rightLowerLeg);
  // right foot
  rightFoot = new Joint(-0.45, 0.15, 0.0, rightLowerLeg);
  myModel.root.children.get(0).children.get(0).children.add(rightFoot);
  // right toes
  rightToes = new Joint(-0.45, 0.15, 1.0, rightFoot);
  myModel.root.children.get(0).children.get(0).children.get(0).children.add(rightToes);
  // left upper leg
  leftUpperLeg = new Joint(0.45, 4.28, 0.0, hips);
  myModel.root.children.add(leftUpperLeg);
  // left lower leg
  leftLowerLeg = new Joint(0.45, 2.265, 0.0, leftUpperLeg);
  myModel.root.children.get(1).children.add(leftLowerLeg);
  // left foot
  leftFoot = new Joint(0.45, 0.15, 0.0, leftLowerLeg);
  myModel.root.children.get(1).children.get(0).children.add(leftFoot);
  // left toes
  leftToes = new Joint(0.45, 0.15, 1.0, leftFoot);
  myModel.root.children.get(1).children.get(0).children.get(0).children.add(leftToes);
  // spine
  spine = new Joint(0.0, 5.3, 0.0, hips);
  myModel.root.children.add(spine);
  // chest
  chest =new Joint(0.0, 6.05, 0.0, spine);
  myModel.root.children.get(2).children.add(chest);
  // upper chest
  upperChest = new Joint(0.0, 6.85, 0.0, chest);
  myModel.root.children.get(2).children.get(0).children.add(upperChest);
  // right shoulder
  rightShoulder = new Joint(-0.645, 7.125, 0.0, upperChest);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(rightShoulder);
  // right upper arm
  rightUpperArm = new Joint(-0.873, 7.125, 0.0, rightShoulder);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.add(rightUpperArm);
  // right lower arm
  rightLowerArm = new Joint(-2.325, 7.125, 0.0, rightUpperArm);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.get(0).children.add(rightLowerArm);
  // right hand
  rightHand = new Joint(-3.625, 7.125, 0.0, rightLowerArm);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(0).children.get(0).children.get(0).children.add(rightHand);
  // left shoulder
  leftShoulder = new Joint(0.645, 7.125, 0.0, upperChest);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(leftShoulder);
  // left upper arm
  leftUpperArm = new Joint(0.873, 7.125, 0.0, leftShoulder);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.add(leftUpperArm);
  // left lower arm
  leftLowerArm = new Joint(2.325, 7.125, 0.0, leftUpperArm);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.get(0).children.add(leftLowerArm);
  // left hand
  leftHand = new Joint(3.625, 7.125, 0.0, leftLowerArm);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(1).children.get(0).children.get(0).children.add(leftHand);
  // neck
  neck = new Joint(0.0, 7.4, 0.0, upperChest);
  myModel.root.children.get(2).children.get(0).children.get(0).children.add(neck);
  // head
  head = new Joint(0.0, 8.12, 0.0, neck);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.add(head);
  // right eye
  rightEye = new Joint(-0.24, 8.25, 0.45, head);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(rightEye);
  // left eye
  leftEye = new Joint(0.24, 8.25, 0.45, head);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(leftEye);
  // jaw
  jaw = new Joint(0.0, 7.595, 0.27, head);
  myModel.root.children.get(2).children.get(0).children.get(0).children.get(2).children.get(0).children.add(jaw);

  // put down the arms
  rotation(rightUpperArm, rightUpperArm.x, rightUpperArm.y, rightUpperArm.z, new Quaternion(cos(90.0 / 2 * PI / 180), 0.0, 0.0, sin(90.0 / 2 * PI / 180)));
  rotation(leftUpperArm, leftUpperArm.x, leftUpperArm.y, leftUpperArm.z, new Quaternion(cos(-90.0 / 2 * PI / 180), 0.0, 0.0, sin(-90.0 / 2 * PI / 180)));
  helperSetModel(rightUpperArm);
  helperSetModel(leftUpperArm);
  translation(hips, -hips.x, -hips.y, -hips.z);
  helperSetModel(hips);
}

/**
 * This is a helper function of setModel() that sets up joints' initial posistions.
 *
 * @param current - the current joint to be handled
 */
void helperSetModel(Joint current) {
  current.ox = current.x;
  current.oy = current.y;
  current.oz = current.z;
  for (Joint j : current.children) {
    helperSetModel(j);
  }
}

/**
 * This function reset the model to its initial position.
 *
 * @param current - the current joint to be reset
 */
void reset(Joint current) {
  current.x = current.ox;
  current.y = current.oy;
  current.z = current.oz;
  current.toTrans = true;
  for (Joint j : current.children) {
    reset(j);
  }
}
