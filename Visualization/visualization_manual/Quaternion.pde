/**
 * File: Quaternion.pde
 *
 * @author Michael Lee, ml3406@rit.edu
 */

/**
 * a class represents quaternion (orientation)
 */
class Quaternion {
  /** scalar */
  public float w;

  /** x component */
  public float x;

  /** y component */
  public float y;

  /** z component */
  public float z;

  /**
   * The constructor initializes all variables.
   */
  public Quaternion(float w, float x, float y, float z) {
    this.w = w;
    this.x = x;
    this.y = y;
    this.z = z;
  }

  /**
   * This function normalizes this quaternion.
   */
  void normalize() {
    float wTemp, xTemp, yTemp, zTemp, difference = 0.000000;
    do {
      wTemp = w / (sqrt(w * w + x * x + y * y + z * z) + difference);
      xTemp = x / (sqrt(w * w + x * x + y * y + z * z) + difference);
      yTemp = y / (sqrt(w * w + x * x + y * y + z * z) + difference);
      zTemp = z / (sqrt(w * w + x * x + y * y + z * z) + difference);
      difference += 0.000001;
    } while (wTemp * wTemp + xTemp * xTemp + yTemp * yTemp + zTemp * zTemp > 1.0);
    w = wTemp;
    x = xTemp;
    y = yTemp;
    z = zTemp;
  }

  /**
   * This function calculates and returns an inverse quaternion of this quaternion.
   */
  Quaternion inverse() {
    return(new Quaternion(w, -x, -y, -z));
  }

  /**
   * This function calculates and returns the production of this quaternion and another one.
   *
   * @param q - the other quaternion
   * @return the production of the quaternions
   */
  Quaternion product(Quaternion q) {
    float wTemp, xTemp, yTemp, zTemp;
    wTemp = w * q.w - x * q.x - y * q.y - z * q.z;
    xTemp = w * q.x + x * q.w + y * q.z - z * q.y;
    yTemp = w * q.y - x * q.z + y * q.w + z * q.x;
    zTemp = w * q.z + x * q.y - y * q.x + z * q.w;
    return new Quaternion(wTemp, xTemp, yTemp, zTemp);
  }
}
