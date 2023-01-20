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

  /** initial x, y, and z positions of this joint */
  public float ox, oy, oz;

  /** a list of childten of this joint */
  public ArrayList<Joint> children;

  /** the parent of this joint */
  public Joint parent;

  /** to perform transformation of this joint or not */
  public boolean toTrans;

  /**
   * The contructor initializes all fields.
   */
  public Joint(float x, float y, float z, Joint parent) {
    index = -1;
    this.x = x;
    this.y = y;
    this.z = z;
    ox = x;
    oy = y;
    oz = z;
    children = new ArrayList();
    toTrans = true;
    this.parent = parent;
  }
}
