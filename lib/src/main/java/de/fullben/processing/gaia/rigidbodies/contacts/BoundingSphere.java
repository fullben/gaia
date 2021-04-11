package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A {@code BoundingSphere} represents a spherical bounding volume that can be tested for overlap.
 *
 * @author Benedikt Full
 */
public class BoundingSphere {

  /** The rigid body encompassed by this bounding sphere. */
  private final RigidBody rigidBody;
  /** The offset of the sphere's center from the {@link #rigidBody}'s position. */
  private final Vector3D offset;
  /** The sphere's radius. */
  private double radius;

  /**
   * Constructs a new bounding sphere based on the given parameters.
   *
   * @param rigidBody the rigid body encompassed by this sphere
   * @param offset the sphere's center's offset from the {@code rigidBody}'s position
   * @param radius the sphere's radius
   */
  public BoundingSphere(RigidBody rigidBody, Vector3D offset, double radius) {
    this.rigidBody = rigidBody;
    this.offset = offset;
    this.radius = Math.abs(radius);
  }

  /**
   * Calculates and returns the volume of the bounding sphere.
   *
   * @return the sphere's volume
   */
  public double getVolume() {
    return (4.0 / 3.0) * Math.PI * (radius * radius * radius);
  }

  /**
   * Checks whether this and the provided bounding sphere overlap. The spheres overlap if the
   * distance of their center points is smaller than their combined radii.
   *
   * @param boundingSphere the bounding sphere with which this sphere might overlap
   * @return true if they overlap, false if not
   */
  public boolean overlaps(BoundingSphere boundingSphere) {
    Vector3D distance = Vector3D.subtract(getCenter(), boundingSphere.getCenter());
    double distanceSquared = distance.squareMagnitude();
    return distanceSquared
        < (radius + boundingSphere.getRadius()) * (radius + boundingSphere.getRadius());
  }

  /**
   * Calculates the current center of the sphere by adding the {@link #offset} to the {@link
   * #rigidBody}'s position and returns the result.
   *
   * @return the sphere's current center
   */
  public Vector3D getCenter() {
    Vector3D center = new Vector3D(rigidBody.getPosition());
    center.add(new Vector3D(offset.getX(), offset.getY(), offset.getZ()));
    return center;
  }

  /**
   * Sets sphere's center to by component.
   *
   * <p>Although this method assumes that the provided parameters represent the absolute center
   * position of the sphere, only the offset from the {@link #rigidBody}'s position is stored.
   *
   * @param x the x coordinate of the center
   * @param y the y coordinate of the center
   * @param z the z coordinate of the center
   */
  public void setCenter(double x, double y, double z) {
    offset.setX(x - rigidBody.getPosition().getX());
    offset.setY(y - rigidBody.getPosition().getY());
    offset.setZ(z - rigidBody.getPosition().getZ());
  }

  /**
   * Returns the offset of the sphere's center from the {@link #rigidBody}'s position.
   *
   * @return the sphere's offset
   */
  public Vector3D getOffset() {
    return offset;
  }

  /**
   * Returns the radius of the sphere.
   *
   * @return the value of {@link #radius}
   */
  public double getRadius() {
    return radius;
  }

  /**
   * Sets the absolute value of the provided double as the sphere's radius.
   *
   * @param radius the new radius
   */
  public void setRadius(double radius) {
    this.radius = Math.abs(radius);
  }
}
