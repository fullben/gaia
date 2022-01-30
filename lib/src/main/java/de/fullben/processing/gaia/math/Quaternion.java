package de.fullben.processing.gaia.math;

/**
 * {@code Quaternion} objects can be used to represent an object's orientation. A quaternion
 * represents an orientation with four values related to the axis and angle in the following way:
 *
 * <pre>
 *   {@code r} = cos (&#x03B8; / 2)
 *   {@code i} = x sin (&#x03B8; / 2)
 *   {@code j} =  y sin (&#x03B8; / 2)
 *   {@code k} = z sin (&#x03B8; / 2)
 * </pre>
 *
 * {@code x}, {@code y} and {@code z} is the axis and &#x03B8; the angle.
 *
 * <p>Quaternions have four degrees of freedom to represent the three degrees of freedom of
 * rotation. In order to constraint away the fourth degree of freedom, a quaternion representing a
 * rotation has to have unit length, or {@code Math.sqrt(r * r + i * i + j * j * k * k) == 1}.
 * Calling {@link #normalize()} enforces this constraint.
 *
 * @author Benedikt Full
 */
public class Quaternion {

  /** The real component of the {@code Quaternion}. */
  private double r;
  /** The first complex component of the {@code Quaternion}. */
  private double i;
  /** The second complex component of the {@code Quaternion}. */
  private double j;
  /** The third complex component of the {@code Quaternion}. */
  private double k;

  /**
   * Constructs a new {@code Quaternion} utilizing the given parameters.
   *
   * @param r real component
   * @param i first complex component
   * @param j second complex component
   * @param k third complex component
   */
  public Quaternion(double r, double i, double j, double k) {
    this.r = r;
    this.i = i;
    this.j = j;
    this.k = k;
  }

  /**
   * Constructs a new quaternion by creating a deep copy of the given quaternion.
   *
   * @param source the quaternion this object will be a copy of
   */
  public Quaternion(Quaternion source) {
    r = source.r;
    i = source.i;
    j = source.j;
    k = source.k;
  }

  /** Constructs a new {@code Quaternion} and sets all its components to zero. */
  public Quaternion() {
    this(0, 0, 0, 0);
  }

  /** Normalizes the quaternion to unit length, making it a valid orientation quaternion. */
  public void normalize() {
    double d = r * r + i * i + j * j + k * k;
    // Check for zero-length and use non-rotation quaternion in that case
    if (d == 0) {
      r = 1;
      return;
    }
    d = 1.0 / Math.sqrt(d);
    r *= d;
    i *= d;
    j *= d;
    k *= d;
  }

  /**
   * Multiplies this quaternion with the provided quaternion. If quaternion {@code a} (this
   * quaternion) and quaternion {@code b} ({@code multiplier}) each represent a rotation, the result
   * of this operation will be equivalent to performing rotation {@code b}, followed by {@code a}.
   *
   * @param multiplier quaternion with which this quaternion will be multiplied with
   */
  public void multiply(Quaternion multiplier) {
    Quaternion tmp = new Quaternion(this);
    r =
        tmp.getR() * multiplier.getR()
            - tmp.getI() * multiplier.getI()
            - tmp.getJ() * multiplier.getJ()
            - tmp.getK() * multiplier.getK();
    i =
        tmp.getR() * multiplier.getI()
            + tmp.getI() * multiplier.getR()
            + tmp.getJ() * multiplier.getK()
            - tmp.getK() * multiplier.getJ();
    j =
        tmp.getR() * multiplier.getJ()
            + tmp.getJ() * multiplier.getR()
            + tmp.getK() * multiplier.getI()
            - tmp.getI() * multiplier.getK();
    k =
        tmp.getR() * multiplier.getK()
            + tmp.getK() * multiplier.getR()
            + tmp.getI() * multiplier.getJ()
            - tmp.getJ() * multiplier.getI();
  }

  /**
   * Rotates this quaternion by the amount specified within the provided vector.
   *
   * @param vec the amount of rotation that will be added to this quaternion
   */
  public void rotateByVector(Vector3D vec) {
    Quaternion multiplier = new Quaternion(0, vec.getX(), vec.getY(), vec.getZ());
    this.multiply(multiplier);
  }

  /**
   * Scales the provided vector by the given scale and adds it to this quaternion. This is used to
   * update the orientation quaternion by a rotation and time.
   *
   * @param vec the vector to add
   * @param scale the amount of the vector to add
   */
  public void addScaledVector(Vector3D vec, double scale) {
    Quaternion q = new Quaternion(0, vec.getX() * scale, vec.getY() * scale, vec.getZ() * scale);
    q.multiply(this);
    r += q.getR() * 0.5;
    i += q.getI() * 0.5;
    j += q.getJ() * 0.5;
    k += q.getK() * 0.5;
  }

  /**
   * Returns the real component of the {@code Quaternion}.
   *
   * @return value of {@link #r}
   */
  public double getR() {
    return r;
  }

  /**
   * Sets the value of the {@code Quaternion}'s real component.
   *
   * @param r the new real component
   */
  public void setR(double r) {
    this.r = r;
  }

  /**
   * Returns the first complex component of the {@code Quaternion}.
   *
   * @return value of {@link #i}
   */
  public double getI() {
    return i;
  }

  /**
   * Sets the value of the {@code Quaternion}'s first complex component.
   *
   * @param i the new first complex component
   */
  public void setI(double i) {
    this.i = i;
  }

  /**
   * Returns the second complex component of the {@code Quaternion}.
   *
   * @return value of {@link #j}
   */
  public double getJ() {
    return j;
  }

  /**
   * Sets the value of the {@code Quaternion}'s second complex component.
   *
   * @param j the second complex component
   */
  public void setJ(double j) {
    this.j = j;
  }

  /**
   * Returns the third complex component of the {@code Quaternion}.
   *
   * @return value of {@link #k}
   */
  public double getK() {
    return k;
  }

  /**
   * Sets the value of the {@code Quaternion}'s third complex component.
   *
   * @param k the third complex component
   */
  public void setK(double k) {
    this.k = k;
  }

  /**
   * Returns the rotation on the x axis represented by this quaternion.
   *
   * @return the roll
   */
  public float x() {
    double sinr = +2.0 * (r * i + j * k);
    double cosr = +1.0 - 2.0 * (i * i + j * j);
    return (float) Math.atan2(sinr, cosr);
  }

  /**
   * Returns the rotation on the y axis represented by this quaternion.
   *
   * @return the pitch
   */
  public float y() {
    double sinp = +2.0 * (r * j - k * i);
    if (Math.abs(sinp) >= 1) {
      return (float) Math.copySign(Math.PI / 2.0, sinp);
    } else {
      return (float) Math.asin(sinp);
    }
  }

  /**
   * Returns the rotation on the z axis represented by this quaternion.
   *
   * @return the yaw
   */
  public float z() {
    double siny = +2.0 * (r * k + i * j);
    double cosy = +1.0 - 2.0 * (j * j + k * k);
    return (float) Math.atan2(siny, cosy);
  }
}
