package de.fullben.processing.gaia.math;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertThrows;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

/**
 * Test class for performing JUnit tests on {@link Vector3D} objects.
 *
 * @author Benedikt Full
 */
public class Vector3DTest {

  @Test
  public void testOrthonormalBase() {
    Vector3D a = new Vector3D(1, 1, 1);
    Vector3D b = new Vector3D(4, 2, 2);
    Vector3D c = new Vector3D(3, 3, 1);
    Vector3D d = new Vector3D(2, 2, 2);
    assertTrue(Vector3D.makeOrthonormalBasis(a, b, c));
    a.normalize();
    b.normalize();
    c.normalize();
    assertEquals(0.0, a.scalarProduct(b));
    assertEquals(0.0, a.scalarProduct(c));
    assertEquals(0.0, b.scalarProduct(c));
    assertFalse(Vector3D.makeOrthonormalBasis(a, d, c));
  }

  @Test
  public void testAddRoutines() {
    Vector3D a = new Vector3D(2, 2, 3);
    Vector3D b = new Vector3D(1, 1.5, -4);
    double resX = 3.0;
    double resY = 3.5;
    double resZ = -1.0;
    // Static routine
    Vector3D result = Vector3D.add(a, b);
    assertEquals(resX, result.getX());
    assertEquals(resY, result.getY());
    assertEquals(resZ, result.getZ());
    // Add to object routine
    a.add(b);
    assertEquals(resX, a.getX());
    assertEquals(resY, a.getY());
    assertEquals(resZ, a.getZ());
    // Add scaled vector to object routine
    double scale = 2.0;
    a.addScaledVector(b, scale);
    assertEquals(resX + b.getX() * scale, a.getX());
    assertEquals(resY + b.getY() * scale, a.getY());
    assertEquals(resZ + b.getZ() * scale, a.getZ());
  }

  @Test
  public void testSubtractionRoutines() {
    Vector3D a = new Vector3D(-1, -.5, 2);
    Vector3D b = new Vector3D(-2, 2, -3);
    double resX = 1.0;
    double resY = -2.5;
    double resZ = 5;
    // Static routine
    Vector3D result = Vector3D.subtract(a, b);
    assertEquals(resX, result.getX());
    assertEquals(resY, result.getY());
    assertEquals(resZ, result.getZ());
    // Add to object routine
    a.subtract(b);
    assertEquals(resX, a.getX());
    assertEquals(resY, a.getY());
    assertEquals(resZ, a.getZ());
  }

  @Test
  public void testMultiplicationRoutines() {
    // Static routine
    Vector3D vec = new Vector3D(0, -1, 50);
    double factor = 3;
    Vector3D result = Vector3D.multiply(vec, factor);
    assertEquals(0, result.getX());
    assertEquals(-3, result.getY());
    assertEquals(150, result.getZ());
    // Multiply on vector
    Vector3D a = new Vector3D(-2, 4, 7);
    a.multiply(-2);
    assertEquals(4, a.getX());
    assertEquals(-8, a.getY());
    assertEquals(-14, a.getZ());
  }

  @Test
  public void testInvert() {
    Vector3D a = new Vector3D(-.5, 2, 0);
    a.invert();
    assertEquals(.5, a.getX());
    assertEquals(-2, a.getY());
    assertEquals(0, a.getZ());
  }

  @Test
  public void testSquaredMagnitude() {
    Vector3D a = new Vector3D(0, -2, 4);
    assertEquals(20, a.squareMagnitude());
  }

  @Test
  public void testMagnitude() {
    Vector3D a = new Vector3D(1, -1, 0);
    assertEquals(Math.sqrt(2), a.magnitude());
    Vector3D b = new Vector3D(0, 0, 0);
    assertEquals(0, b.magnitude());
  }

  @Test
  public void testNormalize() {
    double x = 1;
    double y = -4;
    double z = 2;
    Vector3D a = new Vector3D(x, y, z);
    double magnitude = a.magnitude();
    a.normalize();
    assertEquals(x / magnitude, a.getX());
    assertEquals(y / magnitude, a.getY());
    assertEquals(z / magnitude, a.getZ());
  }

  @Test
  public void testComponentProductRoutines() {
    Vector3D a = new Vector3D(2, 0, -4);
    Vector3D b = new Vector3D(-1, .5, 65);
    double resX = -2;
    double resY = 0;
    double resZ = -260;
    // Static routine
    Vector3D result = Vector3D.componentProduct(a, b);
    assertEquals(resX, result.getX());
    assertEquals(resY, result.getY());
    assertEquals(resZ, result.getZ());
    // Store in object routine
    a.componentProduct(b);
    assertEquals(resX, a.getX());
    assertEquals(resY, a.getY());
    assertEquals(resZ, a.getZ());
  }

  @Test
  public void testScalarProduct() {
    Vector3D a = new Vector3D(0, -5, 2.5);
    Vector3D b = new Vector3D(5, -5, -2);
    double result = a.scalarProduct(b);
    assertEquals(20, result);
  }

  @Test
  public void testVectorProductRoutines() {
    Vector3D a = new Vector3D(0, 5, -5);
    Vector3D b = new Vector3D(-1, -3, 2);
    double resX = -5;
    double resY = 5;
    double resZ = 5;
    // Static routine
    Vector3D result = Vector3D.vectorProduct(a, b);
    assertEquals(resX, result.getX());
    assertEquals(resY, result.getY());
    assertEquals(resZ, result.getZ());
    // Set values in object routine
    a.vectorProduct(b);
    assertEquals(resX, a.getX());
    assertEquals(resY, a.getY());
    assertEquals(resZ, a.getZ());
  }

  @Test
  public void testRandom() {
    Vector3D min;
    Vector3D max;
    for (int i = 0; i < 100; i++) {
      min = new Vector3D(Math.random(), Math.random(), Math.random());
      max = new Vector3D(Math.random(), Math.random(), Math.random());
      Vector3D randomVec = Vector3D.random(min, max);
      // X value
      if (min.getX() > max.getX()) {
        assertTrue(randomVec.getX() < min.getX() && randomVec.getX() > max.getX());
      } else if (min.getX() != randomVec.getX()) {
        assertTrue(randomVec.getX() > min.getX() && randomVec.getX() < max.getX());
      } else {
        assertTrue(randomVec.getX() == min.getX() && randomVec.getX() == max.getX());
      }
      // Y value
      if (min.getY() > max.getY()) {
        assertTrue(randomVec.getY() < min.getY() && randomVec.getY() > max.getY());
      } else if (min.getX() != randomVec.getX()) {
        assertTrue(randomVec.getY() > min.getY() && randomVec.getY() < max.getY());
      } else {
        assertTrue(randomVec.getY() == min.getY() && randomVec.getY() == max.getY());
      }
      // Z value
      if (min.getZ() > max.getZ()) {
        assertTrue(randomVec.getZ() < min.getZ() && randomVec.getZ() > max.getZ());
      } else if (min.getZ() != randomVec.getZ()) {
        assertTrue(randomVec.getZ() > min.getZ() && randomVec.getZ() < max.getZ());
      } else {
        assertTrue(randomVec.getZ() == min.getZ() && randomVec.getZ() == max.getZ());
      }
    }
  }

  @Test
  public void testClear() {
    Vector3D min = new Vector3D(0, 1, -1);
    Vector3D max = new Vector3D(34, 45, 4.5);
    for (int i = 0; i < 100; i++) {
      Vector3D vec = Vector3D.random(min, max);
      vec.clear();
      assertEquals(0, vec.getX());
      assertEquals(0, vec.getY());
      assertEquals(0, vec.getZ());
    }
  }

  @Test
  public void testGetComponent() {
    Vector3D vec = new Vector3D(34, 4, -3);
    assertEquals(34, vec.getComponent(0));
    assertEquals(4, vec.getComponent(1));
    assertEquals(-3, vec.getComponent(2));
    assertThrows(IllegalArgumentException.class, () -> vec.getComponent(-1));
    assertThrows(IllegalArgumentException.class, () -> vec.getComponent(3));
  }

  @Test
  public void testInsertComponent() {
    Vector3D vec = new Vector3D(2, 4, 5);
    assertEquals(2, vec.getX());
    assertEquals(4, vec.getY());
    assertEquals(5, vec.getZ());
    vec.insertComponent(0, 0);
    vec.insertComponent(1, -2);
    vec.insertComponent(2, 1);
    assertEquals(0, vec.getX());
    assertEquals(-2, vec.getY());
    assertEquals(1, vec.getZ());
  }

  @Test
  public void testInvertComponent() {
    Vector3D vec = new Vector3D(0.0, 1, -4);
    assertEquals(0, vec.getX());
    assertEquals(1, vec.getY());
    assertEquals(-4, vec.getZ());
    vec.invertComponent(0);
    vec.invertComponent(1);
    vec.invertComponent(2);
    assertEquals(0, vec.getX());
    assertEquals(-1, vec.getY());
    assertEquals(4, vec.getZ());
  }

  @Test
  public void testSetValuesFromOtherVector() {
    Vector3D vec1 = new Vector3D(1, 2, 3);
    Vector3D vec2 = new Vector3D(4, 5, 6);

    vec2.setValues(vec1);

    assertEquals(1, vec2.getX());
    assertEquals(2, vec2.getY());
    assertEquals(3, vec2.getZ());
  }

  @Test
  public void testSetValuesFromValues() {
    Vector3D vec2 = new Vector3D(4, 5, 6);

    vec2.setValues(1, 2, 3);

    assertEquals(1, vec2.getX());
    assertEquals(2, vec2.getY());
    assertEquals(3, vec2.getZ());
  }

  @Test
  public void testEquals() {
    Vector3D a = new Vector3D(1, 2, 3);
    Vector3D b = new Vector3D(1, -2, 3);
    Vector3D c = new Vector3D(1, 2, 3);
    assertTrue(a.equals(c));
    assertFalse(a.equals(b));
  }

  /**
   * Prints the provided vector's field values to the console.
   *
   * @param vec vector to print
   */
  private void printVecInfo(Vector3D vec) {
    System.out.println("VECTOR VALUES:");
    System.out.println("X: " + vec.getX());
    System.out.println("Y: " + vec.getY());
    System.out.println("Z: " + vec.getZ());
    System.out.println("--------------");
  }
}
