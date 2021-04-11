package de.fullben.processing.gaia.math;

/**
 * {@code Matrix4} objects can be used to represent 3 x 4 matrices. This type of matrix can be used
 * to describe both a rotation and a translation in one matrix. The first three columns of the
 * matrix represent the rotation, the fourth column the position or translation. As operations like
 * multiplication or inversion only work with square matrices, objects of this class are internally
 * treated as homogenous 4 x 4 matrices, with the fourth, additional row containing the values
 * {@code 0 0 0 1}.
 *
 * <p>The twelve fields of the matrix are labeled with alphabetic letters:
 *
 * <pre>
 *   a b c d
 *   e f g h
 *   i j k l
 * </pre>
 *
 * @author Benedikt Full
 */
public class Matrix4 {
  /** Entry of the matrix field located in row one, column one. */
  private double a;
  /** Entry of the matrix field located in row one, column two. */
  private double b;
  /** Entry of the matrix field located in row one, column three. */
  private double c;
  /** Entry of the matrix field located in row one, column four. */
  private double d;
  /** Entry of the matrix field located in row two, column one. */
  private double e;
  /** Entry of the matrix field located in row two, column two. */
  private double f;
  /** Entry of the matrix field located in row two, column three. */
  private double g;
  /** Entry of the matrix field located in row two, column four. */
  private double h;
  /** Entry of the matrix field located in row three, column one. */
  private double i;
  /** Entry of the matrix field located in row three, column two. */
  private double j;
  /** Entry of the matrix field located in row three, column three. */
  private double k;
  /** Entry of the matrix field located in row three, column four. */
  private double l;

  /**
   * Constructs a new matrix based on the given parameters.
   *
   * @param a value for row one, column one
   * @param b value for row one, column two
   * @param c value for row one, column three
   * @param d value for row one, column four
   * @param e value for row two, column one
   * @param f value for row two, column two
   * @param g value for row two, column three
   * @param h value for row two, column four
   * @param i value for row three, column one
   * @param j value for row three, column two
   * @param k value for row three, column three
   * @param l value for row three, column four
   */
  public Matrix4(
      double a,
      double b,
      double c,
      double d,
      double e,
      double f,
      double g,
      double h,
      double i,
      double j,
      double k,
      double l) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.d = d;
    this.e = e;
    this.f = f;
    this.g = g;
    this.h = h;
    this.i = i;
    this.j = j;
    this.k = k;
    this.l = l;
  }

  /**
   * Constructs a new matrix by creating a deep copy of the given matrix.
   *
   * @param source the matrix this object will be the copy of
   */
  public Matrix4(Matrix4 source) {
    a = source.getA();
    b = source.getB();
    c = source.getC();
    d = source.getD();
    e = source.getE();
    f = source.getF();
    g = source.getG();
    h = source.getH();
    i = source.getI();
    j = source.getJ();
    k = source.getK();
    l = source.getL();
  }

  /**
   * Constructs a new identity matrix, containing the following values:
   *
   * <pre>
   *   1 0 0 0
   *   0 1 0 0
   *   0 0 1 0
   * </pre>
   */
  public Matrix4() {
    this(1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);
  }

  /**
   * Multiplies the provided matrices with each other and returns the result as a new matrix object.
   * Multiplying two matrices combines their effect. If the two matrices both represent a
   * transformation, the combined matrix will represent the combined transformation.
   *
   * <p><b>Note:</b> This operation is not commutative - multiplying matrix {@code a} with matrix
   * {@code b} is equal to translating by {@code b} first, followed by {@code a}'s translation.
   *
   * @param a the second translation to be performed
   * @param b the first translation to be performed
   * @return the multiplication's result, a new matrix object
   */
  public static Matrix4 multiply(Matrix4 a, Matrix4 b) {
    return new Matrix4(
        b.getA() * a.getA() + b.getE() * a.getB() + b.getI() * a.getC(),
        b.getB() * a.getA() + b.getF() * a.getB() + b.getJ() * a.getC(),
        b.getC() * a.getA() + b.getG() * a.getB() + b.getK() * a.getC(),
        b.getD() * a.getA() + b.getH() * a.getB() + b.getL() * a.getC() + a.getD(),
        b.getA() * a.getE() + b.getE() * a.getF() + b.getI() * a.getG(),
        b.getB() * a.getE() + b.getF() * a.getF() + b.getJ() * a.getG(),
        b.getC() * a.getE() + b.getG() * a.getF() + b.getK() * a.getG(),
        b.getD() * a.getE() + b.getH() * a.getF() + b.getL() * a.getG() + a.getH(),
        b.getA() * a.getI() + b.getE() * a.getJ() + b.getI() * a.getK(),
        b.getB() * a.getI() + b.getF() * a.getJ() + b.getJ() * a.getK(),
        b.getC() * a.getI() + b.getG() * a.getJ() + b.getK() * a.getK(),
        b.getD() * a.getI() + b.getH() * a.getJ() + b.getL() * a.getK() + a.getL());
  }

  /**
   * Transforms the provided {@code matrix} by {@code vec}. As it is not possible to transform a 3 x
   * 1 vector with a 3 x 4 matrix (vector row count {@literal <} matrix column count), the method
   * internally acts as if the vector has 4 x 1 elements and assumes that the fourth row contains
   * the value one.
   *
   * @param matrix transformation matrix
   * @param vec vector to be transformed
   * @return the result of the transformation as a new vector
   */
  public static Vector3D transform(Matrix4 matrix, Vector3D vec) {
    return new Vector3D(
        vec.getX() * matrix.getA()
            + vec.getY() * matrix.getB()
            + vec.getZ() * matrix.getC()
            + matrix.getD(),
        vec.getX() * matrix.getE()
            + vec.getY() * matrix.getF()
            + vec.getZ() * matrix.getG()
            + matrix.getH(),
        vec.getX() * matrix.getI()
            + vec.getY() * matrix.getJ()
            + vec.getZ() * matrix.getK()
            + matrix.getL());
  }

  /**
   * Transforms the provided vector by this matrix. As it is not possible to transform a 3 x 1
   * vector with a 3 x 4 matrix (vector row count {@literal <} matrix column count), the method
   * internally acts if the vector has 4 x 1 elements and assumes that the fourth row contains the
   * value one.
   *
   * @param vec vector to be transformed
   */
  public void transform(Vector3D vec) {
    double x = vec.getX();
    double y = vec.getY();
    double z = vec.getZ();
    vec.setX(x * a + y * b + z * c + d);
    vec.setY(x * e + y * f + z * g + h);
    vec.setZ(x * i + y * j + z * k + l);
  }

  /**
   * Returns the determinant of this matrix. The inverse of a matrix only exists if its determinant
   * is non-zero.
   *
   * @return the determinant of this matrix
   */
  public double getDeterminant() {
    return -i * f * c + e * j * c + i * b * g - a * j * g - e * b * k + a * f * k;
  }

  /**
   * Sets the elements of this matrix to the inverse of the provided matrix, as long as the
   * determinant of {@code matrix} is not zero. If its determinant actually is zero, the method will
   * have no effect.
   *
   * @param matrix the matrix from which to create the inverse
   */
  public void setInverse(Matrix4 matrix) {
    double det = matrix.getDeterminant();
    // Ensuring determinant is not 0
    if (det == 0.0) {
      return;
    }
    double invDet = 1.0 / det;
    a = (-matrix.getJ() * matrix.getG() + matrix.getF() * matrix.getK()) * invDet;
    e = (matrix.getI() * matrix.getG() - matrix.getE() * matrix.getK()) * invDet;
    i = (-matrix.getI() * matrix.getF() + matrix.getE() * matrix.getJ()) * invDet;

    b = (matrix.getJ() * matrix.getC() - matrix.getB() * matrix.getK()) * invDet;
    f = (-matrix.getI() * matrix.getC() + matrix.getA() * matrix.getK()) * invDet;
    j = (matrix.getI() * matrix.getB() - matrix.getA() * matrix.getJ()) * invDet;

    c = (-matrix.getF() * matrix.getC() + matrix.getB() * matrix.getG()) * invDet;
    g = (+matrix.getE() * matrix.getC() - matrix.getA() * matrix.getG()) * invDet;
    k = (-matrix.getE() * matrix.getB() + matrix.getA() * matrix.getF()) * invDet;

    d =
        (matrix.getJ() * matrix.getG() * matrix.getD()
                - matrix.getF() * matrix.getK() * matrix.getD()
                - matrix.getJ() * matrix.getC() * matrix.getH()
                + matrix.getB() * matrix.getK() * matrix.getH()
                + matrix.getF() * matrix.getC() * matrix.getL()
                - matrix.getB() * matrix.getG() * matrix.getL())
            * invDet;
    h =
        (-matrix.getI() * matrix.getG() * matrix.getD()
                + matrix.getE() * matrix.getK() * matrix.getD()
                + matrix.getI() * matrix.getC() * matrix.getH()
                - matrix.getA() * matrix.getK() * matrix.getH()
                - matrix.getE() * matrix.getC() * matrix.getL()
                + matrix.getA() * matrix.getG() * matrix.getL())
            * invDet;
    l =
        (matrix.getI() * matrix.getF() * matrix.getD()
                - matrix.getE() * matrix.getJ() * matrix.getD()
                - matrix.getI() * matrix.getB() * matrix.getH()
                + matrix.getA() * matrix.getJ() * matrix.getH()
                + matrix.getE() * matrix.getB() * matrix.getL()
                - matrix.getA() * matrix.getF() * matrix.getL())
            * invDet;
  }

  /**
   * Creates a new matrix representing the inverse of this matrix. If the determinant of this matrix
   * is zero, all elements of the returned matrix will be zero.
   *
   * @return the inverse of this matrix
   */
  public Matrix4 inverse() {
    Matrix4 inverse = new Matrix4();
    inverse.setInverse(this);
    return inverse;
  }

  /** Inverts the matrix. */
  public void invert() {
    Matrix4 inverse = new Matrix4();
    inverse.setInverse(this);
    a = inverse.getA();
    b = inverse.getB();
    c = inverse.getC();
    d = inverse.getD();
    e = inverse.getE();
    f = inverse.getF();
    g = inverse.getG();
    h = inverse.getH();
    i = inverse.getI();
    j = inverse.getJ();
    k = inverse.getK();
    l = inverse.getL();
  }

  /**
   * Sets this matrix to represent both the rotation of the provided quaternion and the translation
   * described by the given vector.
   *
   * @param o the new orientation
   * @param pos the new position
   */
  public void setOrientationAndPosition(Quaternion o, Vector3D pos) {
    a = 1 - (2 * o.getJ() * o.getJ() + 2 * o.getK() * o.getK());
    b = 2 * o.getI() * o.getJ() + 2 * o.getK() * o.getR();
    c = 2 * o.getI() * o.getK() - 2 * o.getJ() * o.getR();
    d = pos.getX();

    e = 2 * o.getI() * o.getJ() - 2 * o.getK() * o.getR();
    f = 1 - (2 * o.getI() * o.getI() + 2 * o.getK() * o.getK());
    g = 2 * o.getI() * o.getK() + 2 * o.getJ() * o.getR();
    h = pos.getY();

    i = 2 * o.getI() * o.getK() + 2 * o.getJ() * o.getR();
    j = 2 * o.getJ() * o.getK() - 2 * o.getI() * o.getR();
    k = 1 - (2 * o.getI() * o.getI() + 2 * o.getJ() * o.getJ());
    l = pos.getZ();
  }

  /**
   * Transforms the provided vector by the transformational inverse of this matrix.
   *
   * @param vec the vector to be transformed
   * @return the result of the transformation
   */
  public Vector3D transformInverse(Vector3D vec) {
    Vector3D tmp = new Vector3D(vec);
    tmp.setX(tmp.getX() - c);
    tmp.setY(tmp.getY() - h);
    tmp.setZ(tmp.getZ() - l);
    return new Vector3D(
        tmp.getX() * a + tmp.getY() * e + tmp.getZ() * i,
        tmp.getX() * b + tmp.getY() * f + tmp.getZ() * j,
        tmp.getX() * c + tmp.getY() * g + tmp.getZ() * k);
  }

  /**
   * Transforms the given direction vector by this matrix and returns the resulting vector.
   *
   * @param vec the vector to be transformed
   * @return the result of the transformation
   */
  public Vector3D transformDirection(Vector3D vec) {
    return new Vector3D(
        vec.getX() * a + vec.getY() * b + vec.getZ() * c,
        vec.getX() * e + vec.getY() * f + vec.getZ() * g,
        vec.getX() * i + vec.getY() * j + vec.getZ() * k);
  }

  /**
   * Transform the given direction vector by the transformational inverse of this matrix.
   *
   * @param vec the vector to be transformed
   * @return the result of the transformation
   */
  public Vector3D transformInverseDirection(Vector3D vec) {
    return new Vector3D(
        vec.getX() * a + vec.getY() * e + vec.getZ() * i,
        vec.getX() * b + vec.getY() * f + vec.getZ() * j,
        vec.getX() * c + vec.getY() * g + vec.getZ() * k);
  }

  /**
   * Returns any of the matrix's four columns as a vector. This method will throw a {@code
   * NullPointerException} if the provided index is invalid.
   *
   * @param index the column index, valid range is 0 to 3
   * @return the axis vector specified by the {@code index}
   */
  public Vector3D getAxisVector(int index) {
    if (index >= 0 && index <= 3) {
      if (index == 0) {
        return new Vector3D(a, e, i);
      } else if (index == 1) {
        return new Vector3D(b, f, j);
      } else if (index == 2) {
        return new Vector3D(c, g, k);
      } else {
        return new Vector3D(d, h, l);
      }
    } else {
      throw new NullPointerException("Invalid index");
    }
  }

  /**
   * Returns the value located at row one, column one of this matrix.
   *
   * @return the value of {@link #a}
   */
  public double getA() {
    return a;
  }

  /**
   * Sets the value of the matrix field located at row one, column one to the given value.
   *
   * @param a the new value of {@link #a}
   */
  public void setA(double a) {
    this.a = a;
  }

  /**
   * Returns the value located at row one, column two of this matrix.
   *
   * @return the value of {@link #b}
   */
  public double getB() {
    return b;
  }

  /**
   * Sets the value of the matrix field located at row one, column two to the given value.
   *
   * @param b the new value of {@link #b}
   */
  public void setB(double b) {
    this.b = b;
  }

  /**
   * Returns the value located at row one, column three of this matrix.
   *
   * @return the value of {@link #c}
   */
  public double getC() {
    return c;
  }

  /**
   * Sets the value of the matrix field located at row one, column three to the given value.
   *
   * @param c the new value of {@link #c}
   */
  public void setC(double c) {
    this.c = c;
  }

  /**
   * Returns the value located at row one, column four of this matrix.
   *
   * @return the value of {@link #d}
   */
  public double getD() {
    return d;
  }

  /**
   * Sets the value of the matrix field located at row one, column four to the given value.
   *
   * @param d the new value of {@link #d}
   */
  public void setD(double d) {
    this.d = d;
  }

  /**
   * Returns the value located at row two, column one of this matrix.
   *
   * @return the value of {@link #e}
   */
  public double getE() {
    return e;
  }

  /**
   * Sets the value of the matrix field located at row two, column one to the given value.
   *
   * @param e the new value of {@link #e}
   */
  public void setE(double e) {
    this.e = e;
  }

  /**
   * Returns the value located at row two, column two of this matrix.
   *
   * @return the value of {@link #f}
   */
  public double getF() {
    return f;
  }

  /**
   * Sets the value of the matrix field located at row two, column two to the given value.
   *
   * @param f the new value of {@link #f}
   */
  public void setF(double f) {
    this.f = f;
  }

  /**
   * Returns the value located at row two, column three of this matrix.
   *
   * @return the value of {@link #g}
   */
  public double getG() {
    return g;
  }

  /**
   * Sets the value of the matrix field located at row two, column three to the given value.
   *
   * @param g the new value of {@link #g}
   */
  public void setG(double g) {
    this.g = g;
  }

  /**
   * Returns the value located at row two, column four of this matrix.
   *
   * @return the value of {@link #h}
   */
  public double getH() {
    return h;
  }

  /**
   * Sets the value of the matrix field located at row two, column four to the given value.
   *
   * @param h the new value of {@link #h}
   */
  public void setH(double h) {
    this.h = h;
  }

  /**
   * Returns the value located at row three, column one of this matrix.
   *
   * @return the value of {@link #i}
   */
  public double getI() {
    return i;
  }

  /**
   * Sets the value of the matrix field located at row three, column one to the given value.
   *
   * @param i the new value of {@link #i}
   */
  public void setI(double i) {
    this.i = i;
  }

  /**
   * Returns the value located at row three, column two of this matrix.
   *
   * @return the value of {@link #j}
   */
  public double getJ() {
    return j;
  }

  /**
   * Sets the value of the matrix field located at row three, column two to the given value.
   *
   * @param j the new value of {@link #j}
   */
  public void setJ(double j) {
    this.j = j;
  }

  /**
   * Returns the value located at row three, column three of this matrix.
   *
   * @return the value of {@link #k}
   */
  public double getK() {
    return k;
  }

  /**
   * Sets the value of the matrix field located at row three, column three to the given value.
   *
   * @param k the new value of {@link #k}
   */
  public void setK(double k) {
    this.k = k;
  }

  /**
   * Returns the value located at row three, column four of this matrix.
   *
   * @return the value of {@link #l}
   */
  public double getL() {
    return l;
  }

  /**
   * Sets the value of the matrix field located at row three, column four to the given value.
   *
   * @param l the new value of {@link #l}
   */
  public void setL(double l) {
    this.l = l;
  }
}
