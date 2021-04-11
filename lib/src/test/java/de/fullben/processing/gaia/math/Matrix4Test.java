package de.fullben.processing.gaia.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.fail;

import org.junit.jupiter.api.Test;

/**
 * Test class for verifying the methods of {@link Matrix4}.
 *
 * @author Benedikt Full
 */
public class Matrix4Test {

  @Test
  public void testMultiply() {
    Matrix4 a = new Matrix4(1, 2, 3, 4, 5, 6, 2, 4, 3, 7, 0, 2);
    Matrix4 b = new Matrix4(3, 4, 2, 6, 7, 8, 1, 2, 3, 5, 0, 4);
    Matrix4 res = Matrix4.multiply(a, b);
    assertEquals(26, res.getA());
    assertEquals(35, res.getB());
    assertEquals(4, res.getC());
    assertEquals(26, res.getD());
    assertEquals(63, res.getE());
    assertEquals(78, res.getF());
    assertEquals(16, res.getG());
    assertEquals(54, res.getH());
    assertEquals(58, res.getI());
    assertEquals(68, res.getJ());
    assertEquals(13, res.getK());
    assertEquals(34, res.getL());
  }

  @Test
  public void testTransformationRoutines() {
    Matrix4 matrix = new Matrix4(4, 5, 6, 1, 2, 3, 8, 7, 2, 4, 1, 4);
    Vector3D vec = new Vector3D(8, 2, 4);
    // Static routine
    Vector3D res = Matrix4.transform(matrix, vec);
    assertEquals(67, res.getX());
    assertEquals(61, res.getY());
    assertEquals(32, res.getZ());
    // Routine changing provided vector object
    matrix.transform(vec);
    assertEquals(res.getX(), vec.getX());
    assertEquals(res.getY(), vec.getY());
    assertEquals(res.getZ(), vec.getZ());
  }

  @Test
  public void testGetDeterminant() {
    Matrix4 a = new Matrix4(2, 4, 6, 8, 10, 1, 3, 5, 7, 2, 3, 5);
    Matrix4 b = new Matrix4(4, 1, 3, 7, 9, 10, 1, 4, 2, 8, 9, 1);
    assertEquals(36, a.getDeterminant());
    assertEquals(405, b.getDeterminant());
  }

  @Test
  public void testInverseRoutines() {
    Matrix4 m = new Matrix4(2, 4, 6, 8, 2, 3, 6, 7, 1, 3, 4, 5);
    Matrix4 inv = new Matrix4();
    // Storing inverse of m in inv
    inv.setInverse(m);
    assertEquals(3, inv.getA());
    assertEquals(-1, inv.getB());
    assertEquals(-3, inv.getC());
    assertEquals(-2, inv.getD());
    assertEquals(1, inv.getE());
    assertEquals(-1, inv.getF());
    assertEquals(-0.0, inv.getG());
    assertEquals(-1, inv.getH());
    assertEquals(-1.5, inv.getI());
    assertEquals(1, inv.getJ());
    assertEquals(1, inv.getK());
    assertEquals(-0.0, inv.getL());
    // Returning new matrix containing inverse
    Matrix4 mInv = m.inverse();
    assertEquals(inv.getA(), mInv.getA());
    assertEquals(inv.getB(), mInv.getB());
    assertEquals(inv.getC(), mInv.getC());
    assertEquals(inv.getD(), mInv.getD());
    assertEquals(inv.getE(), mInv.getE());
    assertEquals(inv.getF(), mInv.getF());
    assertEquals(inv.getG(), mInv.getG());
    assertEquals(inv.getH(), mInv.getH());
    assertEquals(inv.getI(), mInv.getI());
    assertEquals(inv.getJ(), mInv.getJ());
    assertEquals(inv.getK(), mInv.getK());
    assertEquals(inv.getL(), mInv.getL());
    // Setting matrix to its inverse
    m.invert();
    assertEquals(inv.getA(), m.getA());
    assertEquals(inv.getB(), m.getB());
    assertEquals(inv.getC(), m.getC());
    assertEquals(inv.getD(), m.getD());
    assertEquals(inv.getE(), m.getE());
    assertEquals(inv.getF(), m.getF());
    assertEquals(inv.getG(), m.getG());
    assertEquals(inv.getH(), m.getH());
    assertEquals(inv.getI(), m.getI());
    assertEquals(inv.getJ(), m.getJ());
    assertEquals(inv.getK(), m.getK());
    assertEquals(inv.getL(), m.getL());
  }

  @Test
  public void testSetOrientationAndPosition() {
    Matrix4 matrix = new Matrix4();
    Quaternion o = new Quaternion(1, 3, -4, 2);
    Vector3D pos = new Vector3D(3, -1, 4);
    matrix.setOrientationAndPosition(o, pos);
    assertEquals(-39, matrix.getA());
    assertEquals(-20, matrix.getB());
    assertEquals(20, matrix.getC());
    assertEquals(3, matrix.getD());
    assertEquals(-28, matrix.getE());
    assertEquals(-25, matrix.getF());
    assertEquals(4, matrix.getG());
    assertEquals(-1, matrix.getH());
    assertEquals(4, matrix.getI());
    assertEquals(-22, matrix.getJ());
    assertEquals(-49, matrix.getK());
    assertEquals(4, matrix.getL());
  }

  @Test
  public void testTransformInverse() {
    Matrix4 matrix = new Matrix4(2, -1, 2, 3, 7, 0, 1, 3, 2, 4, 5, 2);
    Vector3D vec = new Vector3D(-2, 4, 6);
    Vector3D res = matrix.transformInverse(vec);
    assertEquals(7, res.getX());
    assertEquals(20, res.getY());
    assertEquals(13, res.getZ());
  }

  @Test
  public void testTransformDirection() {
    Matrix4 matrix = new Matrix4(2, -4, 1, 3, 7, 5, 8, -1, 5, 0, 3, 2);
    Vector3D vec = new Vector3D(-2, 6, 1);
    Vector3D res = matrix.transformDirection(vec);
    assertEquals(-27, res.getX());
    assertEquals(24, res.getY());
    assertEquals(-7, res.getZ());
  }

  @Test
  public void testTransformInverseDirection() {
    Matrix4 matrix = new Matrix4(6, -8, 3, 4, 7, 5, 1, -1, 2, 5, 8, 1);
    Vector3D vec = new Vector3D(2, -3, 6);
    Vector3D res = matrix.transformInverseDirection(vec);
    assertEquals(3, res.getX());
    assertEquals(-1, res.getY());
    assertEquals(51, res.getZ());
  }

  @Test
  public void testGetAxisVector() {
    Matrix4 matrix = new Matrix4(6, 2, -3, 5, 5.5, 0, -1, 2, 3, 56, 6, 9);
    Vector3D col0 = matrix.getAxisVector(0);
    assertEquals(6, col0.getX());
    assertEquals(5.5, col0.getY());
    assertEquals(3, col0.getZ());
    Vector3D col1 = matrix.getAxisVector(1);
    assertEquals(2, col1.getX());
    assertEquals(0, col1.getY());
    assertEquals(56, col1.getZ());
    Vector3D col2 = matrix.getAxisVector(2);
    assertEquals(-3, col2.getX());
    assertEquals(-1, col2.getY());
    assertEquals(6, col2.getZ());
    Vector3D col3 = matrix.getAxisVector(3);
    assertEquals(5, col3.getX());
    assertEquals(2, col3.getY());
    assertEquals(9, col3.getZ());
    try {
      matrix.getAxisVector(-1);
    } catch (NullPointerException npe) {
      // Exception expected
      return;
    }
    fail();
  }
}
