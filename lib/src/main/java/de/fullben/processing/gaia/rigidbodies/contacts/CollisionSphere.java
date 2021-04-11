package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Matrix4;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * Spherical collision geometry capable of representing a part of a rigid body or a rigid body that
 * can be treated as a sphere for collision detection.
 *
 * @author Benedikt Full
 */
public class CollisionSphere extends CollisionGeometry {

  /** The radius of the sphere. */
  private double radius;

  /**
   * Constructs a new collision sphere based on the given parameters.
   *
   * @param rigidBody the rigid body (partially) represented by this sphere
   * @param offset the offset of the sphere's center from the body's position
   * @param radius the sphere's radius
   */
  public CollisionSphere(RigidBody rigidBody, Matrix4 offset, double radius) {
    super(rigidBody, offset);
    this.radius = radius;
  }

  /**
   * Constructs a new collision sphere based on the given parameters. It is assumed that this
   * sphere's center is located at the rigid body's position.
   *
   * @param rigidBody the rigid body (partially) represented by this sphere
   * @param radius the sphere's radius
   */
  public CollisionSphere(RigidBody rigidBody, double radius) {
    super(rigidBody);
    this.radius = radius;
  }

  /**
   * Calculates and returns the volume of this collision sphere.
   *
   * @return the sphere's volume
   */
  @Override
  public double getVolume() {
    return (4.0 / 3.0) * Math.PI * (radius * radius * radius);
  }

  /**
   * Returns the radius of the collision sphere.
   *
   * @return the value of {@link #radius}
   */
  public double getRadius() {
    return radius;
  }

  /**
   * Sets the sphere's radius to the given parameter.
   *
   * @param radius the sphere's new radius
   */
  public void setRadius(double radius) {
    this.radius = radius;
  }
}
