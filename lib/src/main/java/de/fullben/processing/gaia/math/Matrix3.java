package de.fullben.processing.gaia.math;

/**
 * {@code Matrix3} objects can be used to represent 3 x 3 matrices. Their main purpose is to
 * represent a transformation in 3D space that does not include a translational component. See
 * {@link Matrix4} for the combination of movement and rotation.
 *
 * <p>The nine fields of the matrix are labeled with alphabetic letters:
 *
 * <pre>
 *   a b c
 *   d e f
 *   g h i
 * </pre>
 *
 * @author Benedikt Full
 */
public class Matrix3 {

  /** Entry of the matrix field located in row one, column one. */
  private double a;
  /** Entry of the matrix field located in row one, column two. */
  private double b;
  /** Entry of the matrix field located in row one, column three. */
  private double c;
  /** Entry of the matrix field located in row two, column one. */
  private double d;
  /** Entry of the matrix field located in row two, column two. */
  private double e;
  /** Entry of the matrix field located in row two, column three. */
  private double f;
  /** Entry of the matrix field located in row three, column one. */
  private double g;
  /** Entry of the matrix field located in row three, column two. */
  private double h;
  /** Entry of the matrix field located in row three, column three. */
  private double i;

  /**
   * Constructs a new matrix based on the given parameters.
   *
   * @param a value for row one, column one
   * @param b value for row one, column two
   * @param c value for row one, column three
   * @param d value for row two, column one
   * @param e value for row two, column two
   * @param f value for row two, column three
   * @param g value for row three, column one
   * @param h value for row three, column two
   * @param i value for row three, column three
   */
  public Matrix3(
      double a, double b, double c, double d, double e, double f, double g, double h, double i) {
    this.a = a;
    this.b = b;
    this.c = c;
    this.d = d;
    this.e = e;
    this.f = f;
    this.g = g;
    this.h = h;
    this.i = i;
  }

  /**
   * Constructs a new matrix by creating a deep copy of the given matrix.
   *
   * @param source the matrix this object will be the copy of
   */
  public Matrix3(Matrix3 source) {
    a = source.getA();
    b = source.getB();
    c = source.getC();
    d = source.getD();
    e = source.getE();
    f = source.getF();
    g = source.getG();
    h = source.getH();
    i = source.getI();
  }

  /** Constructs a new matrix and sets all nine fields to zero. */
  public Matrix3() {
    this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
  }

  /**
   * Interpolates the two provided matrices linearly and returns the result.
   *
   * <p>The amount of matrix {@code a} and {@code b} used for interpolation is controlled by the
   * value of {@code prop}, which specifies the proportion of matrix {@code b}. The proportion of
   * matrix {@code a} is calculated by subtracting {@code prop} from 1.
   *
   * @param a the first matrix
   * @param b the second matrix
   * @param prop the proportion of matrix {@code b}, should honor {@code 0 < prop < 1}
   * @return a new matrix with the interpolation result
   */
  public static Matrix3 linearInterpolate(Matrix3 a, Matrix3 b, double prop) {
    Matrix3 result = new Matrix3();
    double omp = 1.0 - prop;
    result.setA(a.getA() * omp + b.getA() * prop);
    result.setB(a.getB() * omp + b.getB() * prop);
    result.setC(a.getC() * omp + b.getC() * prop);
    result.setD(a.getD() * omp + b.getD() * prop);
    result.setE(a.getE() * omp + b.getE() * prop);
    result.setF(a.getF() * omp + b.getF() * prop);
    result.setG(a.getG() * omp + b.getG() * prop);
    result.setH(a.getH() * omp + b.getH() * prop);
    result.setI(a.getI() * omp + b.getI() * prop);
    return result;
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
  public static Matrix3 multiply(Matrix3 a, Matrix3 b) {
    return new Matrix3(
        a.getA() * b.getA() + a.getB() * b.getD() + a.getC() * b.getG(),
        a.getA() * b.getB() + a.getB() * b.getE() + a.getC() * b.getH(),
        a.getA() * b.getC() + a.getB() * b.getF() + a.getC() * b.getI(),
        a.getD() * b.getA() + a.getE() * b.getD() + a.getF() * b.getG(),
        a.getD() * b.getB() + a.getE() * b.getE() + a.getF() * b.getH(),
        a.getD() * b.getC() + a.getE() * b.getF() + a.getF() * b.getI(),
        a.getG() * b.getA() + a.getH() * b.getD() + a.getI() * b.getG(),
        a.getG() * b.getB() + a.getH() * b.getE() + a.getI() * b.getH(),
        a.getG() * b.getC() + a.getH() * b.getF() + a.getI() * b.getI());
  }

  /**
   * Transforms the vector by the matrix.
   *
   * @param matrix the matrix by which the vector will be transformed
   * @param vec the vector to be transformed
   * @return the resulting new vector
   */
  public static Vector3D transform(Matrix3 matrix, Vector3D vec) {
    return new Vector3D(
        vec.getX() * matrix.getA() + vec.getY() * matrix.getB() + vec.getZ() * matrix.getC(),
        vec.getX() * matrix.getD() + vec.getY() * matrix.getE() + vec.getZ() * matrix.getF(),
        vec.getX() * matrix.getG() + vec.getY() * matrix.getH() + vec.getZ() * matrix.getI());
  }

  /**
   * Transforms the provided vector by this matrix.
   *
   * @param vec the vector to be transformed
   */
  public void transform(Vector3D vec) {
    double x = vec.getX();
    double y = vec.getY();
    double z = vec.getZ();
    vec.setX(x * a + y * b + z * c);
    vec.setY(x * d + y * e + z * f);
    vec.setZ(x * g + y * h + z * i);
  }

  /**
   * Multiplies this and the provided {@code matrix} and stores the result in this object.
   * Multiplying two matrices combines their effect. If the two matrices both represent a
   * transformation, the combined matrix will represent the combined transformation.
   *
   * <p><b>Note:</b> Matrix multiplication is not commutative - multiplying this matrix with {@code
   * matrix} means performing the translation described by {@code matrix} first, followed by the
   * translation described by this matrix.
   *
   * @param matrix the matrix with which this matrix will be multiplied with
   */
  public void multiply(Matrix3 matrix) {
    double t1;
    double t2;
    double t3;
    t1 = a * matrix.getA() + b * matrix.getD() + c * matrix.getG();
    t2 = a * matrix.getB() + b * matrix.getE() + c * matrix.getH();
    t3 = a * matrix.getC() + b * matrix.getF() + c * matrix.getI();
    a = t1;
    b = t2;
    c = t3;
    t1 = d * matrix.getA() + e * matrix.getD() + f * matrix.getG();
    t2 = d * matrix.getB() + e * matrix.getE() + f * matrix.getH();
    t3 = d * matrix.getC() + e * matrix.getF() + f * matrix.getI();
    d = t1;
    e = t2;
    f = t3;
    t1 = g * matrix.getA() + h * matrix.getD() + i * matrix.getG();
    t2 = g * matrix.getB() + h * matrix.getE() + i * matrix.getH();
    t3 = g * matrix.getC() + h * matrix.getF() + i * matrix.getI();
    g = t1;
    h = t2;
    i = t3;
  }

  /**
   * Multiplies this matrix with the provided factor. The multiplication is achieved by multiplying
   * each of the matrix's individual components with the provided factor.
   *
   * @param factor the factor with which the matrix will be multiplied with
   */
  public void multiply(double factor) {
    a *= factor;
    b *= factor;
    c *= factor;
    d *= factor;
    e *= factor;
    f *= factor;
    g *= factor;
    h *= factor;
    i *= factor;
  }

  /**
   * Returns the determinant of this matrix. The inverse of a matrix only exists if its determinant
   * is non-zero.
   *
   * @return the determinant of this matrix
   */
  public double getDeterminant() {
    return a * e * i - a * f * h - b * d * i + c * d * h + b * g * f - c * g * e;
  }

  /**
   * Sets this matrix to the inverse of the provided matrix, as long as the matrix is invertible
   * (determinant is non-zero).
   *
   * @param matrix the matrix for which to create the inverse
   */
  public void setInverse(Matrix3 matrix) {
    double t1 = matrix.getA() * matrix.getE();
    double t2 = matrix.getA() * matrix.getF();
    double t3 = matrix.getB() * matrix.getD();
    double t4 = matrix.getC() * matrix.getD();
    double t5 = matrix.getB() * matrix.getG();
    double t6 = matrix.getC() * matrix.getG();

    // Determinant
    double det = matrix.getDeterminant();
    // Ensuring determinant is not zero
    if (det == 0.0) {
      return;
    }
    double invDet = 1.0 / det;

    a = (matrix.getE() * matrix.getI() - matrix.getF() * matrix.getH()) * invDet;
    b = -(matrix.getB() * matrix.getI() - matrix.getC() * matrix.getH()) * invDet;
    c = (matrix.getB() * matrix.getF() - matrix.getC() * matrix.getE()) * invDet;
    d = -(matrix.getD() * matrix.getI() - matrix.getF() * matrix.getG()) * invDet;
    e = (matrix.getA() * matrix.getI() - t6) * invDet;
    f = -(t2 - t4) * invDet;
    g = (matrix.getD() * matrix.getH() - matrix.getE() * matrix.getG()) * invDet;
    h = -(matrix.getA() * matrix.getH() - t5) * invDet;
    i = (t1 - t3) * invDet;
  }

  /**
   * Creates a new matrix representing the inverse of this matrix. If the determinant of this matrix
   * is zero, all elements of the returned matrix will be zero.
   *
   * @return the inverse of this matrix
   */
  public Matrix3 inverse() {
    Matrix3 inverse = new Matrix3();
    inverse.setInverse(this);
    return inverse;
  }

  /** Inverts the matrix. */
  public void invert() {
    Matrix3 inverse = new Matrix3();
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
  }

  /**
   * Sets this matrix as the transpose of the provided {@code matrix}. The transpose of a matrix is
   * created by swapping its rows and columns. This operation can be useful whenever the matrix
   * represents a rotation only.
   *
   * @param matrix the matrix from which to create the transpose
   */
  public void setTranspose(Matrix3 matrix) {
    a = matrix.getA();
    b = matrix.getD();
    c = matrix.getG();
    d = matrix.getB();
    e = matrix.getE();
    f = matrix.getH();
    g = matrix.getC();
    h = matrix.getF();
    i = matrix.getI();
  }

  /**
   * Creates the transpose of this matrix. The transpose of a matrix is created by swapping its rows
   * and columns. This operation can be useful whenever the matrix represents a rotation only.
   *
   * @return a matrix containing the transpose of this matrix
   */
  public Matrix3 transpose() {
    Matrix3 transpose = new Matrix3();
    transpose.setTranspose(this);
    return transpose;
  }

  /**
   * Transforms the provided vector by the transpose matrix of this matrix and returns the result as
   * a new vector.
   *
   * @param vector the vector to be transformed
   * @return a new, transformed vector
   */
  public Vector3D transformTranspose(Vector3D vector) {
    Vector3D transform = new Vector3D(vector);
    // Create transpose of this matrix
    Matrix3 transpose = new Matrix3();
    transpose.setTranspose(this);
    // Transform vector by transpose
    transpose.transform(transform);
    return transform;
  }

  /**
   * Sets this matrix to be the rotation matrix corresponding to the given quaternion.
   *
   * @param o an orientation
   */
  public void setOrientation(Quaternion o) {
    a = 1 - (2 * o.getJ() * o.getJ() + 2 * o.getK() * o.getK());
    b = 2 * o.getI() * o.getJ() + 2 * o.getK() * o.getR();
    c = 2 * o.getI() * o.getK() - 2 * o.getJ() * o.getR();
    d = 2 * o.getI() * o.getJ() - 2 * o.getK() * o.getR();
    e = 1 - (2 * o.getI() * o.getI() + 2 * o.getK() * o.getK());
    f = 2 * o.getJ() * o.getK() + 2 * o.getI() * o.getR();
    g = 2 * o.getI() * o.getK() + 2 * o.getJ() * o.getR();
    h = 2 * o.getJ() * o.getK() - 2 * o.getI() * o.getR();
    i = 1 - (2 * o.getI() * o.getI() + 2 * o.getJ() * o.getJ());
  }

  /** Sets the value of the matrix from inertia tensor values. */
  private void setInertiaTensorCoefficients(
      double ix, double iy, double iz, double ixy, double ixz, double iyz) {
    a = ix;
    b = -ixy;
    c = -ixz;
    d = -ixy;
    e = iy;
    f = -iyz;
    g = ixz;
    h = -iyz;
    i = iz;
  }

  private void setInertiaTensorCoefficients(double x, double y, double z) {
    a = x;
    b = -(x * y);
    c = -(x * z);
    d = -(x * y);
    e = y;
    f = -(y * z);
    g = x * z;
    h = -(y * z);
    i = z;
  }

  /**
   * Sets the value of the matrix as an inertia tensor of a rectangular block aligned with the
   * body's coordinate system with the given axis half-sizes and mass.
   *
   * <p>Source: https://github.com/stojg/vector/blob/master/vector.go
   *
   * @param halfSizes the axis half-sizes
   * @param mass the mass value
   */
  public void setBlockInertiaTensor(Vector3D halfSizes, double mass) {
    Vector3D squares = Vector3D.componentProduct(halfSizes, halfSizes);
    setInertiaTensorCoefficients(
        0.3 * mass * (squares.getY() + squares.getZ()),
        0.3 * mass * (squares.getX() + squares.getZ()),
        0.3 * mass * (squares.getX() + squares.getY()),
        0,
        0,
        0);
  }

  /**
   * Sets the matrix values from the given three vector's components. These are arranged as the
   * three columns of the matrix.
   *
   * @param colOne the first column
   * @param colTwo the second column
   * @param colThree the third column
   */
  public void setComponents(Vector3D colOne, Vector3D colTwo, Vector3D colThree) {
    a = colOne.getX();
    b = colTwo.getX();
    c = colThree.getX();
    d = colOne.getY();
    e = colTwo.getY();
    f = colThree.getY();
    g = colOne.getZ();
    h = colTwo.getZ();
    i = colThree.getZ();
  }

  /**
   * Sets this matrix as the skew-symmetric matrix of the provided vector. A skew-symmetric matrix's
   * transpose equals its negative.
   *
   * <p>Multiplying (transforming) a vector {@code a} with the skew-symmetric matrix {@code mb} of
   * vector {@code b} yields the same result as performing the cross product operation with vector
   * {@code a} and {@code b}:
   *
   * <p>{@code a} × {@code b} = {@code mb} * {@code a}
   *
   * <p>The matrix is made up of the vector components the following way:
   *
   * <pre>
   *    0 -z  y
   *    z  0 -x
   *   -y  x  0
   * </pre>
   *
   * @param vector the vector for which the skew-symmetric matrix will be build
   */
  public void setSkewSymmetric(Vector3D vector) {
    a = 0;
    b = -vector.getZ();
    c = vector.getY();
    d = vector.getZ();
    e = 0;
    f = -vector.getX();
    g = -vector.getY();
    h = vector.getX();
    i = 0;
  }

  /**
   * Adds the provided matrix to this matrix by adding up their corresponding components.
   *
   * @param matrix the matrix to be added to this matrix
   */
  public void add(Matrix3 matrix) {
    a += matrix.getA();
    b += matrix.getB();
    c += matrix.getC();
    d += matrix.getD();
    e += matrix.getE();
    f += matrix.getF();
    g += matrix.getG();
    h += matrix.getH();
    i += matrix.getI();
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
   * Returns the value located at row two, column one of this matrix.
   *
   * @return the value of {@link #d}
   */
  public double getD() {
    return d;
  }

  /**
   * Sets the value of the matrix field located at row two, column one to the given value.
   *
   * @param d the new value of {@link #d}
   */
  public void setD(double d) {
    this.d = d;
  }

  /**
   * Returns the value located at row two, column two of this matrix.
   *
   * @return the value of {@link #e}
   */
  public double getE() {
    return e;
  }

  /**
   * Sets the value of the matrix field located at row two, column two to the given value.
   *
   * @param e the new value of {@link #e}
   */
  public void setE(double e) {
    this.e = e;
  }

  /**
   * Returns the value located at row two, column three of this matrix.
   *
   * @return the value of {@link #f}
   */
  public double getF() {
    return f;
  }

  /**
   * Sets the value of the matrix field located at row two, column three to the given value.
   *
   * @param f the new value of {@link #f}
   */
  public void setF(double f) {
    this.f = f;
  }

  /**
   * Returns the value located at row three, column one of this matrix.
   *
   * @return the value of {@link #g}
   */
  public double getG() {
    return g;
  }

  /**
   * Sets the value of the matrix field located at row three, column one to the given value.
   *
   * @param g the new value of {@link #g}
   */
  public void setG(double g) {
    this.g = g;
  }

  /**
   * Returns the value located at row three, column two of this matrix.
   *
   * @return the value of {@link #h}
   */
  public double getH() {
    return h;
  }

  /**
   * Sets the value of the matrix field located at row three, column three to the given value.
   *
   * @param h the new value of {@link #h}
   */
  public void setH(double h) {
    this.h = h;
  }

  /**
   * Returns the value located at row three, column three of this matrix.
   *
   * @return the value of {@link #i}
   */
  public double getI() {
    return i;
  }

  /**
   * Sets the value of the matrix field located at row three, column three to the given value.
   *
   * @param i the new value of {@link #i}
   */
  public void setI(double i) {
    this.i = i;
  }
}
