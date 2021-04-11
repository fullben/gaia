package de.fullben.processing.gaia.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

/**
 * Test class for verifying the methods of {@link Matrix3}.
 *
 * @author Benedikt Full
 */
public class Matrix3Test {

  @Test
  public void testMultiplicationRoutines() {
    Matrix3 a = new Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3 b = new Matrix3(2, 4, 8, 16, 2, 4, 8, 16, 2);
    Matrix3 res = Matrix3.multiply(a, b);
    assertEquals(58, res.getA());
    assertEquals(56, res.getB());
    assertEquals(22, res.getC());
    assertEquals(136, res.getD());
    assertEquals(122, res.getE());
    assertEquals(64, res.getF());
    assertEquals(214, res.getG());
    assertEquals(188, res.getH());
    assertEquals(106, res.getI());
    a.multiply(b);
    assertEquals(res.getA(), a.getA());
    assertEquals(res.getB(), a.getB());
    assertEquals(res.getC(), a.getC());
    assertEquals(res.getD(), a.getD());
    assertEquals(res.getE(), a.getE());
    assertEquals(res.getF(), a.getF());
    assertEquals(res.getG(), a.getG());
    assertEquals(res.getH(), a.getH());
    assertEquals(res.getI(), a.getI());
    Matrix3 c = new Matrix3(3, 4, 5, 2, 0, -3, 4, -3, 2);
    c.multiply(-5);
    assertEquals(-15, c.getA());
    assertEquals(-20, c.getB());
    assertEquals(-25, c.getC());
    assertEquals(-10, c.getD());
    assertEquals(-0.0, c.getE());
    assertEquals(15, c.getF());
    assertEquals(-20, c.getG());
    assertEquals(15, c.getH());
    assertEquals(-10, c.getI());
  }

  @Test
  public void testTransformationRoutines() {
    Matrix3 matrix = new Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Vector3D vec = new Vector3D(2, 4, 1);
    Vector3D res = Matrix3.transform(matrix, vec);
    assertEquals(13, res.getX());
    assertEquals(34, res.getY());
    assertEquals(55, res.getZ());
    matrix.transform(vec);
    assertEquals(res.getX(), vec.getX());
    assertEquals(res.getY(), vec.getY());
    assertEquals(res.getZ(), vec.getZ());
  }

  @Test
  public void testGetDeterminant() {
    Matrix3 a = new Matrix3(1, 2, 3, 1, 4, 6, 2, 7, 8);
    assertEquals(-5, a.getDeterminant());
    Matrix3 b = new Matrix3(-1, -2, 5, 2, 3, 7, 1, -7, 0);
    assertEquals(-148, b.getDeterminant());
  }

  @Test
  public void testInverseRoutines() {
    Matrix3 matrix = new Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 8);
    Matrix3 matrix1 = new Matrix3();
    // Storing inverse in other matrix object
    matrix1.setInverse(matrix);
    assertEquals(-8.0 / 3.0, matrix1.getA());
    assertEquals(8.0 / 3.0, matrix1.getB());
    assertEquals(-1, matrix1.getC());
    assertEquals(3.333333333333333, matrix1.getD());
    assertEquals(-13.0 / 3.0, matrix1.getE());
    assertEquals(2, matrix1.getF());
    assertEquals(-1, matrix1.getG());
    assertEquals(2, matrix1.getH());
    assertEquals(-1, matrix1.getI());
    // Turning matrix into its own inverse
    matrix.invert();
    assertEquals(matrix.getA(), matrix1.getA());
    assertEquals(matrix.getB(), matrix1.getB());
    assertEquals(matrix.getC(), matrix1.getC());
    assertEquals(matrix.getD(), matrix1.getD());
    assertEquals(matrix.getE(), matrix1.getE());
    assertEquals(matrix.getF(), matrix1.getF());
    assertEquals(matrix.getG(), matrix1.getG());
    assertEquals(matrix.getH(), matrix1.getH());
    assertEquals(matrix.getI(), matrix1.getI());
  }

  @Test
  public void testTransposeRoutines() {
    Matrix3 matrix = new Matrix3(1, 2, 3, 4, 5, 6, 7, 8, 9);
    Matrix3 transpose = new Matrix3();
    transpose.setTranspose(matrix);
    assertEquals(1, transpose.getA());
    assertEquals(4, transpose.getB());
    assertEquals(7, transpose.getC());
    assertEquals(2, transpose.getD());
    assertEquals(5, transpose.getE());
    assertEquals(8, transpose.getF());
    assertEquals(3, transpose.getG());
    assertEquals(6, transpose.getH());
    assertEquals(9, transpose.getI());
    Matrix3 m = matrix.transpose();
    assertEquals(transpose.getA(), m.getA());
    assertEquals(transpose.getB(), m.getB());
    assertEquals(transpose.getC(), m.getC());
    assertEquals(transpose.getD(), m.getD());
    assertEquals(transpose.getE(), m.getE());
    assertEquals(transpose.getF(), m.getF());
    assertEquals(transpose.getG(), m.getG());
    assertEquals(transpose.getH(), m.getH());
    assertEquals(transpose.getI(), m.getI());
  }

  @Test
  public void testSetOrientation() {
    Matrix3 matrix = new Matrix3();
    Quaternion o = new Quaternion(8, 4, 2, 3);
    matrix.setOrientation(o);
    assertEquals(-25, matrix.getA());
    assertEquals(64, matrix.getB());
    assertEquals(-8, matrix.getC());
    assertEquals(-32, matrix.getD());
    assertEquals(-49, matrix.getE());
    assertEquals(76, matrix.getF());
    assertEquals(56, matrix.getG());
    assertEquals(-52, matrix.getH());
    assertEquals(-39, matrix.getI());
  }

  @Test
  public void testSetSkewSymmetric() {
    Matrix3 matrix = new Matrix3();
    Vector3D vec = new Vector3D(-1, .5, 5);
    matrix.setSkewSymmetric(vec);
    assertEquals(0, matrix.getA());
    assertEquals(-5, matrix.getB());
    assertEquals(.5, matrix.getC());
    assertEquals(5, matrix.getD());
    assertEquals(0, matrix.getE());
    assertEquals(1, matrix.getF());
    assertEquals(-.5, matrix.getG());
    assertEquals(-1, matrix.getH());
    assertEquals(0, matrix.getI());
  }

  @Test
  public void testAdd() {
    Matrix3 a = new Matrix3(1, 2, 3, 456, 93, 381, -13, 3, 1);
    Matrix3 b = new Matrix3(43, -5, 21, 1, -3, 34, 4, 2, 3);
    a.add(b);
    assertEquals(44, a.getA());
    assertEquals(-3, a.getB());
    assertEquals(24, a.getC());
    assertEquals(457, a.getD());
    assertEquals(90, a.getE());
    assertEquals(415, a.getF());
    assertEquals(-9, a.getG());
    assertEquals(5, a.getH());
    assertEquals(4, a.getI());
  }
}
