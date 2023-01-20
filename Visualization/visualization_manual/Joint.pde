/**
 * File: Joint.pde
 *
 * @author Michael Lee, ml3406@rit.edu
 */

/**
 * a class represents joints
 */
class Joint {
  /** the index of this joint for accessing motion data */
  public int index;

  /** x, y, and z positions of this joint */
  public float x, y, z;

  /** a list of childten of this joint */
  public ArrayList<Joint> children;

  /**
   * The contructor initializes all fields.
   */
  public Joint(float x, float y, float z, int index) {
    this.index = index;
    this.x = x;
    this.y = y;
    this.z = z;
    children = new ArrayList();
  }
}
