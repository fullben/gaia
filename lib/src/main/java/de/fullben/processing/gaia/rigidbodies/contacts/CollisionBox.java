package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Matrix4;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * Collision box, capable of representing a part of a rigid body or a rigid body that can be treated
 * as an aligned bounding box for collision detection.
 *
 * @author Benedikt Full
 */
public class CollisionBox extends CollisionGeometry {

  /** The half-sizes of the box along each of its local axes. */
  private Vector3D halfSize;

  /**
   * Constructs a new collision box based on the given parameters.
   *
   * @param rigidBody the rigid body (partially) represented by this box
   * @param offset the offset of the box's center from the body's position
   * @param halfSize the half-sizes of the box along each of its local axes
   */
  public CollisionBox(RigidBody rigidBody, Matrix4 offset, Vector3D halfSize) {
    super(rigidBody, offset);
    this.halfSize = halfSize;
  }

  /**
   * Constructs a new collision box based on the given parameters. It is assumed that this box's
   * center is located at the rigid body's position.
   *
   * @param rigidBody the rigid body (partially) represented by this box
   * @param halfSize the half-sizes of the box along each of its local axes
   */
  public CollisionBox(RigidBody rigidBody, Vector3D halfSize) {
    super(rigidBody);
    this.halfSize = halfSize;
  }

  /**
   * Calculates and returns the volume of this collision box.
   *
   * @return the box's volume
   */
  @Override
  public double getVolume() {
    return Math.abs((2 * halfSize.getX()) * (2 * halfSize.getY()) * (2 * halfSize.getZ()));
  }

  /**
   * Returns the vector containing the half-sizes of the box along each of its local axes.
   *
   * @return {@link #halfSize}
   */
  public Vector3D getHalfSize() {
    return halfSize;
  }

  /**
   * Sets the box's half-sizes to the given vector.
   *
   * @param halfSize the box's new half-sizes
   */
  public void setHalfSize(Vector3D halfSize) {
    this.halfSize = halfSize;
  }
}
