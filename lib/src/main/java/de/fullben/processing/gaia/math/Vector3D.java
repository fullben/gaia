package de.fullben.processing.gaia.math;

import de.fullben.processing.gaia.Configuration;
import java.util.Objects;

/**
 * A vector class for representing quantities (e.g. velocity, acceleration or position) in 3D space
 * with a left-handed coordinate system.
 *
 * @author Benedikt Full
 */
public class Vector3D {

  /** This field holds a quantity on the x axis of 3D space. */
  private double x;
  /** This field holds a quantity on the y axis of 3D space. */
  private double y;
  /** This field holds a quantity on the z axis of 3D space. */
  private double z;

  /**
   * Constructs a new vector and sets its coordinates to the provided values.
   *
   * @param x a quantity on the x axis of 3D space
   * @param y a quantity on the y axis of 3D space
   * @param z a quantity on the z axis of 3D space
   */
  public Vector3D(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  /**
   * Constructs a new vector by creating a deep copy of the given vector.
   *
   * @param source the vector this object will be a copy of
   */
  public Vector3D(Vector3D source) {
    x = source.x;
    y = source.y;
    z = source.z;
  }

  /**
   * Constructs a zero vector; the {@link #x}, {@link #y} and {@link #z} fields will be initialized
   * with a value of {@code 0}.
   */
  public Vector3D() {
    this(0, 0, 0);
  }

  /**
   * Returns a new vector which is the sum of both provided vectors. Vectors addition is performed
   * by adding the corresponding components of the vectors.
   *
   * @param vecOne the first of the two summands
   * @param vecTwo the second of the two summands
   * @return a new {@code Vector3D} holding the result of the addition of both provided vectors
   */
  public static Vector3D add(Vector3D vecOne, Vector3D vecTwo) {
    return new Vector3D(vecOne.x + vecTwo.x, vecOne.y + vecTwo.y, vecOne.z + vecTwo.z);
  }

  /**
   * Subtracts the second provided vector from the first and returns the resulting new vector.
   * Vector subtraction is performed by subtracting the corresponding components of the subtrahend
   * from the components of the minuend.
   *
   * @param minuend vector from which the second provided vector will be subtracted
   * @param subtrahend vector to subtract
   * @return a new {@code Vector3D} holding the result of the subtraction
   */
  public static Vector3D subtract(Vector3D minuend, Vector3D subtrahend) {
    return new Vector3D(
        minuend.x - subtrahend.x, minuend.y - subtrahend.y, minuend.z - subtrahend.z);
  }

  /**
   * Multiplies the vector with the provided factor by multiplying the vector's individual
   * components with the {@code factor}. The method returns the result as a new vector.
   *
   * @param vec the vector that will be multiplied
   * @param factor the scalar/factor with which the vector will be multiplied
   * @return a new {@code Vector3D} holding the result of the multiplication
   */
  public static Vector3D multiply(Vector3D vec, double factor) {
    return new Vector3D(vec.x * factor, vec.y * factor, vec.z * factor);
  }

  /**
   * Creates the component product of the two provided vectors by multiplying their corresponding
   * components and returning a new vector with the results.
   *
   * <p>The component product is one of the three multiplication operations that can be performed on
   * vectors. It is a commutative operation.
   *
   * @param vecOne the first of the two factors
   * @param vecTwo the second of the two factors
   * @return a new {@code Vector3D} holding the result of the component product operation
   */
  public static Vector3D componentProduct(Vector3D vecOne, Vector3D vecTwo) {
    return new Vector3D(vecOne.x * vecTwo.x, vecOne.y * vecTwo.y, vecOne.z * vecTwo.z);
  }

  /**
   * Calculates the vector product (also known as cross product) of the two provided vectors.
   *
   * <p>The vector product is one of the three multiplication operations that can be performed on
   * vectors. It is a non-commutative operation. The vector created as result of this operation is
   * at right angles to both of its operands.
   *
   * @param vecOne the first of the two vectors
   * @param vecTwo the second of the two vectors
   * @return a vector at right angles to both provided vectors
   */
  public static Vector3D vectorProduct(Vector3D vecOne, Vector3D vecTwo) {
    return new Vector3D(
        vecOne.y * vecTwo.z - vecOne.z * vecTwo.y,
        vecOne.z * vecTwo.x - vecOne.x * vecTwo.z,
        vecOne.x * vecTwo.y - vecOne.y * vecTwo.x);
  }

  /**
   * Attempts to construct a triple of mutually orthogonal vectors (all three vectors are at a right
   * angle to each other). The first two provided vectors {@code vecOne} and {@code vecTwo} must not
   * be parallel to each other, as this would mean that there is not a unique vector that is
   * orthogonal to these two, but infinite vectors. If this case occurs, this method will return
   * false and none of the provided vectors will be altered.
   *
   * <p><b>Note:</b> If this method is able to construct an orthogonal triple, it will simply update
   * the vectors found under the three provided vector references instead of returning new {@code
   * Vector3D} objects.
   *
   * @param vecOne a vector not parallel to {@code vecTwo}
   * @param vecTwo a vector not parallel to {@code vecOne}
   * @param vecThree a third vector
   * @return true if the method was able to construct three orthogonal vectors, false if not
   */
  public static boolean makeOrthonormalBasis(Vector3D vecOne, Vector3D vecTwo, Vector3D vecThree) {
    Vector3D a = new Vector3D(vecOne.x, vecOne.y, vecOne.z);
    Vector3D c = new Vector3D(vecThree.x, vecThree.y, vecThree.z);
    vecTwo.normalize();
    vecThree.x = vecTwo.x;
    vecThree.y = vecTwo.y;
    vecThree.z = vecTwo.z;
    vecThree.vectorProduct(vecOne);
    if (vecThree.squareMagnitude() == 0.0) {
      // vecOne and vecTwo are parallel - no way to generate orthogonal basis, resetting vectors
      vecOne.x = a.x;
      vecOne.y = a.y;
      vecOne.z = a.z;
      vecThree.x = c.x;
      vecThree.y = c.y;
      vecThree.z = c.z;
      return false;
    }
    vecOne.normalize();
    vecTwo.x = vecOne.x;
    vecTwo.y = vecOne.y;
    vecTwo.z = vecOne.z;
    vecTwo.vectorProduct(vecThree);
    return true;
  }

  /**
   * Generates a random vector with values located within the range defined by the two provided
   * vectors.
   *
   * <p>The possible values of the returned vector do not include the defined upper and lower bound
   * unless the upper and lower bound values are equal. If a minimum value is larger than its
   * corresponding maximum value, the maximum is treated as the lower bound and the minimum as the
   * upper.
   *
   * @param min the range's lower bound
   * @param max the range's upper bound
   * @return a vector located within the range defined by {@code min} and {@code max}
   */
  public static Vector3D random(Vector3D min, Vector3D max) {
    Vector3D ranVector = new Vector3D();
    double lowerBound = min.x;
    double upperBound = max.x;
    ranVector.x = random(lowerBound, upperBound);
    lowerBound = min.y;
    upperBound = max.y;
    ranVector.y = random(lowerBound, upperBound);
    lowerBound = min.z;
    upperBound = max.z;
    ranVector.z = random(lowerBound, upperBound);
    return ranVector;
  }

  /**
   * Returns a random number located within the range defined by {@code min} and {@code max}.
   *
   * <p>The possible return values do not include the lower and upper range bound unless these
   * bounds are equal to each other. If {@code min} is larger than {@code max}, it will be treated
   * as the range's upper bound and {@code max} as the range's lower bound.
   *
   * @param min the range's lower bound
   * @param max the range's upper bound
   * @return a value located within the range defined by {@code min} and {@code max}
   */
  private static double random(double min, double max) {
    if (min == max) {
      return min;
    } else {
      if (min > max) {
        double tmp = min;
        min = max;
        max = tmp;
      }
      double range = max - min;
      return Math.random() * range + min;
    }
  }

  /** Flips all the components of the vector. */
  public void invert() {
    x = -x + 0.0;
    y = -y + 0.0;
    z = -z + 0.0;
  }

  /**
   * Calculates the straight-line distance of the change of the vector.
   *
   * @return a positive value or 0
   * @see #squareMagnitude()
   */
  public double magnitude() {
    return Math.sqrt(squareMagnitude());
  }

  /**
   * Calculates the squared magnitude of the vector. This is faster than {@link #magnitude()} as it
   * avoids calling {@link Math#sqrt(double)}. The value returned by this method can be used when
   * comparing the magnitude of two vectors, as comparing the squared magnitude of two vectors with
   * each other will always return the same result as comparing their actual magnitudes.
   *
   * @return a positive value or 0
   */
  public double squareMagnitude() {
    return x * x + y * y + z * z;
  }

  /**
   * Turns any non-zero vector into a vector of unit length. The vector has the length 1 and holds
   * the direction of the change represented by the vector.
   */
  public void normalize() {
    double magnitude = magnitude();
    if (magnitude > 0) {
      x /= magnitude;
      y /= magnitude;
      z /= magnitude;
    }
  }

  /**
   * Multiplies this vector with the provided factor. This is achieved by multiplying the vector's
   * individual components with the provided {@code factor}.
   *
   * @param factor the scalar/factor with which the vector will be multiplied
   */
  public void multiply(double factor) {
    x *= factor;
    y *= factor;
    z *= factor;
  }

  /**
   * Multiplies this vector with the provided vector. Vectors addition is performed by adding the
   * corresponding components of the vectors.
   *
   * @param vec the vector that will be added to this vector
   */
  public void add(Vector3D vec) {
    x += vec.x;
    y += vec.y;
    z += vec.z;
  }

  /**
   * Subtracts the provided vector from this vector. Vector subtraction is performed by subtracting
   * the corresponding components of the subtrahend from the components of the minuend.
   *
   * @param subtrahend the vector that will be subtracted from this vector
   */
  public void subtract(Vector3D subtrahend) {
    x -= subtrahend.x;
    y -= subtrahend.y;
    z -= subtrahend.z;
  }

  /**
   * Multiplies the provided vector with the also provided factor before adding it to this vector.
   *
   * @param vec the vector that will be multiplied and added to this vector
   * @param scale the factor with which {@code vec} will be multiplied with
   */
  public void addScaledVector(Vector3D vec, double scale) {
    x += vec.x * scale;
    y += vec.y * scale;
    z += vec.z * scale;
  }

  /**
   * Creates the component product of this and the provided vector by multiplying their
   * corresponding components and storing the result in this object.
   *
   * <p>The component product is one of the three multiplication operations that can be performed on
   * vectors. It is a commutative operation.
   *
   * @param vec the vector this vector will be multiplied with
   */
  public void componentProduct(Vector3D vec) {
    x *= vec.x;
    y *= vec.y;
    z *= vec.z;
  }

  /**
   * Calculates and returns the scalar product of this and the provided vector. This is achieved by
   * multiplying the vectors' corresponding components and adding up the three resulting numbers.
   *
   * <p>The scalar product is one of the three multiplication operations that can be performed on
   * vectors. It is a commutative operation and by far the most common multiplication operation used
   * when working with vectors.
   *
   * @param vec the second vector used for the scalar product
   * @return the scalar product, a number
   */
  public double scalarProduct(Vector3D vec) {
    return x * vec.x + y * vec.y + z * vec.z;
  }

  /**
   * Calculates the vector product (also known as cross product) of this and the provided vector and
   * stores the result in this object.
   *
   * <p>The vector product is one of the three multiplication operations that can be performed on
   * vectors. It is a non-commutative operation. The vector created as result of this operation is
   * at right angles to both of its operands.
   *
   * @param vec the second vector object
   */
  public void vectorProduct(Vector3D vec) {
    double newX = y * vec.z - z * vec.y;
    double newY = z * vec.x - x * vec.z;
    double newZ = x * vec.y - y * vec.x;
    x = newX;
    y = newY;
    z = newZ;
  }

  /** Clears the vector by setting its {@code x}, {@code y} and {@code z} value to 0. */
  public void clear() {
    x = 0;
    y = 0;
    z = 0;
  }

  /**
   * Compares this and the provided vector's {@code x}, {@code y} and {@code z} components and
   * returns true if they all are equal.
   *
   * @param vector the vector with which this vector will be compared
   * @return true if the component values of this vector and the provided vector are the same
   */
  public boolean equals(Vector3D vector) {
    return x == vector.x && y == vector.y && z == vector.z;
  }

  /**
   * Returns the vector component found at the provided index. This method will throw a {@code
   * NullPointerException} if the provided index is invalid.
   *
   * @param index the component's index: 0 for {@code x}, 1 for {@code y} and 2 for {@code z}
   * @return the {@code x}, {@code y} or {@code z} component of this vector
   */
  public double getComponent(int index) {
    if (index == 0) {
      return x;
    } else if (index == 1) {
      return y;
    } else if (index == 2) {
      return z;
    } else {
      throw new IllegalArgumentException("Invalid index, valid are 0, 1, 2");
    }
  }

  /**
   * Inserts the provided value at the component identified by the given index. This method has no
   * effect when the given index is not a valid value.
   *
   * @param index the component's index: 0 for {@code x}, 1 for {@code y} and 2 for {@code z}
   * @param value the new value of the component identified by the index
   */
  public void insertComponent(int index, double value) {
    if (index == 0) {
      x = value;
    } else if (index == 1) {
      y = value;
    } else if (index == 2) {
      z = value;
    }
  }

  /**
   * Inverts the vector component found at the provided index. This method has no effect when the
   * given index is not a valid value.
   *
   * @param index the component's index: 0 for {@code x}, 1 for {@code y} and 2 for {@code z}
   */
  public void invertComponent(int index) {
    if (index == 0) {
      x = -x + 0.0;
    } else if (index == 1) {
      y = -y + 0.0;
    } else if (index == 2) {
      z = -z + 0.0;
    }
  }

  /**
   * Sets the x, y and z values of this vector to the corresponding values of the given vector.
   *
   * @param vec the vector of which the values will be copied to this vector
   */
  public void setValues(Vector3D vec) {
    x = vec.x;
    y = vec.y;
    z = vec.z;
  }

  /**
   * Sets the x, y and z values of this vector to the given values.
   *
   * @param x the new x value
   * @param y the new y value
   * @param z the new z value
   */
  public void setValues(double x, double y, double z) {
    this.x = x;
    this.y = y;
    this.z = z;
  }

  /**
   * Returns the quantity this vector has on the x axis.
   *
   * <p><b>Note:</b> Do not use {@code getX()} when attempting to utilize vector values in {@code
   * Processing}. Instead, use {@link #x()}.
   *
   * @return the value of {@link #x}
   */
  public double getX() {
    return x;
  }

  /**
   * * Sets the value of {@link #x} to the provided value.
   *
   * @param x new quantity on the x axis
   */
  public void setX(double x) {
    this.x = x;
  }

  /**
   * Returns the quantity this vector has on the y axis.
   *
   * <p><b>Note:</b> Do not use {@code getY()} when attempting to utilize vector values in {@code
   * Processing}. Instead, use {@link #y()}.
   *
   * @return the value of {@link #y}
   */
  public double getY() {
    return y;
  }

  /**
   * Sets the value of {@link #y} to the provided value.
   *
   * @param y new quantity on the y axis
   */
  public void setY(double y) {
    this.y = y;
  }

  /**
   * Returns the quantity this vector has on the z axis.
   *
   * <p><b>Note:</b> Do not use {@code getZ()} when attempting to utilize vector values in {@code
   * Processing}. Instead, use {@link #z()}.
   *
   * @return the value of {@link #z}
   */
  public double getZ() {
    return z;
  }

  /**
   * Sets the value of {@link #z} to the provided value.
   *
   * @param z new quantity on the z axis
   */
  public void setZ(double z) {
    this.z = z;
  }

  /**
   * Returns a scaled float version of the vector's {@code x} value. This value can be used for
   * positioning an object within {@code Processing}'s pixel-based coordinate system.
   *
   * <p><b>Note:</b> Do not use {@link #getX()} when attempting to utilize vector values in {@code
   * Processing}.
   *
   * @return a scaled version of the vector's quantity on the {@code x} axis
   */
  public float x() {
    return (float) (x * Configuration.getConfig().getProcessingFactor());
  }

  /**
   * Returns a scaled float version of the vector's {@code y} value. This value can be used for
   * positioning an object within {@code Processing}'s pixel-based coordinate system.
   *
   * <p><b>Note:</b> Do not use {@link #getY()} when attempting to utilize vector values in {@code
   * Processing}.
   *
   * @return a scaled version of the vector's quantity on the {@code y} axis
   */
  public float y() {
    return (float) (y * Configuration.getConfig().getProcessingFactor());
  }

  /**
   * Returns a scaled float version of the vector's {@code z} value. This value can be used for
   * positioning an object within {@code Processing}'s pixel-based coordinate system.
   *
   * <p><b>Note:</b> Do not use {@link #getZ()} when attempting to utilize vector values in {@code
   * Processing}.
   *
   * @return a scaled version of the vector's quantity on the {@code z} axis
   */
  public float z() {
    return (float) (z * Configuration.getConfig().getProcessingFactor());
  }

  @Override
  public boolean equals(Object o) {
    if (this == o) {
      return true;
    }
    if (o == null || getClass() != o.getClass()) {
      return false;
    }
    Vector3D vector3D = (Vector3D) o;
    return Double.compare(vector3D.x, x) == 0
        && Double.compare(vector3D.y, y) == 0
        && Double.compare(vector3D.z, z) == 0;
  }

  @Override
  public int hashCode() {
    return Objects.hash(x, y, z);
  }
}
