package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;

/**
 * The collision plane does not represent a rigid body. It is used for contacts with the immovable
 * world geometry.
 *
 * <p><b>Note:</b> As this class does not actually represent a rigid body, it does not extend the
 * collision geometry base class, {@link CollisionGeometry}.
 *
 * @author Benedikt Full
 */
public class CollisionPlane {

  /** The plane normal. */
  private Vector3D direction;
  /** The distance of the plane from the origin. */
  private double originOffset;

  /**
   * Constructs a new collision plane based on the given parameters.
   *
   * @param direction the plane's normal vector
   * @param originOffset the distance between the origin and the plane
   */
  public CollisionPlane(Vector3D direction, double originOffset) {
    this.direction = direction;
    this.originOffset = originOffset;
  }

  /**
   * Returns the plane's normal vector.
   *
   * @return {@link #direction}
   */
  public Vector3D getDirection() {
    return direction;
  }

  /**
   * Sets the provided vector as the plane's normal vector.
   *
   * @param direction the plane's normal vector
   */
  public void setDirection(Vector3D direction) {
    this.direction = direction;
  }

  /**
   * Returns the distance between the origin and the plane.
   *
   * @return the value of {@link #originOffset}
   */
  public double getOriginOffset() {
    return originOffset;
  }

  /**
   * Sets the given value as the distance between the origin and the plane.
   *
   * @param originOffset the distance between the origin and the plane
   */
  public void setOriginOffset(double originOffset) {
    this.originOffset = originOffset;
  }
}
