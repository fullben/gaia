package de.fullben.processing.gaia.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;

import org.junit.jupiter.api.Test;

/**
 * Test class for verifying the methods of {@link Quaternion}.
 *
 * @author Benedikt Full
 */
public class QuaternionTest {

  @Test
  public void testNormalize() {
    Quaternion q = new Quaternion(); // Determinant will be zero
    q.normalize();
    assertEquals(1, q.getR()); // r is set to one if determinant is zero
    assertEquals(0, q.getI());
    assertEquals(0, q.getJ());
    assertEquals(0, q.getK());
    q = new Quaternion(1, 2, 4, 5);
  }

  @Test
  public void testMultiply() {
    Quaternion a = new Quaternion(2, 4, 8, 1);
    Quaternion b = new Quaternion(1, 3, 7, 2);
    a.multiply(b);
    assertEquals(-68, a.getR());
    assertEquals(19, a.getI());
    assertEquals(17, a.getJ());
    assertEquals(9, a.getK());
  }

  @Test
  public void testRotateByVector() {
    Quaternion q = new Quaternion(1, 2, 3, 4);
    Quaternion a = new Quaternion(q);
    Vector3D vec = new Vector3D(2, 4, 8);
    q.rotateByVector(vec);
    // Fail if quaternion method required for performing test does not pass its own tests
    testMultiply();
    // Calculating expected result
    Quaternion b = new Quaternion(0, vec.getX(), vec.getY(), vec.getZ());
    a.multiply(b);
    assertEquals(a.getR(), q.getR());
    assertEquals(a.getI(), q.getI());
    assertEquals(a.getJ(), q.getJ());
    assertEquals(a.getK(), q.getK());
  }

  @Test
  public void testAddScaledVector() {
    Vector3D vec = new Vector3D(2, 4, 8);
    double scale = 3.0;
    Quaternion q = new Quaternion(1, 2, 3, 4);
    // Fail if quaternion method required for performing test does not pass its own tests
    testMultiply();
    Quaternion a = new Quaternion(q);
    Quaternion b = new Quaternion(0, vec.getX() * scale, vec.getY() * scale, vec.getZ() * scale);
    q.addScaledVector(vec, scale);
    b.multiply(a);
    assertEquals(a.getR() + b.getR() * 0.5, q.getR());
    assertEquals(a.getI() + b.getI() * 0.5, q.getI());
    assertEquals(a.getJ() + b.getJ() * 0.5, q.getJ());
    assertEquals(a.getK() + b.getK() * 0.5, q.getK());
  }

  @Test
  public void testCopy() {
    Quaternion q = new Quaternion(1, 2, 3, 4);
    Quaternion c = new Quaternion(q);
    // Not the same object
    assertFalse(q.equals(c));
    // But same values
    assertEquals(q.getR(), c.getR());
    assertEquals(q.getI(), c.getI());
    assertEquals(q.getJ(), c.getJ());
    assertEquals(q.getK(), c.getK());
  }
}
