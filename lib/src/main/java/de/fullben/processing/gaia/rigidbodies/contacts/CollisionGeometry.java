package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Matrix4;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * Represents a primitive collision geometry object which may be utilized to detect collisions
 * against.
 *
 * @author Benedikt Full
 */
public abstract class CollisionGeometry {

  /** The rigid rigidBody that is (partially) represented by this collision geometry object. */
  private RigidBody rigidBody;
  /**
   * The offset (translation and rotation) of this primitive from the {@link #rigidBody}'s position.
   */
  private Matrix4 offset;
  /**
   * The resultant transform of the primitive collision geometry. This is calculated by combining
   * the offset of the primitive with the transform of the rigid body.
   */
  private Matrix4 transform;

  /**
   * Constructs a new collision geometry object (partially) representing the given rigid body. The
   * object is offset from the body by the translation and rotation represented by the provided
   * matrix.
   *
   * @param rigidBody the rigid body
   * @param offset the offset of the geometrical shape from the body's position
   */
  public CollisionGeometry(RigidBody rigidBody, Matrix4 offset) {
    this.rigidBody = rigidBody;
    this.offset = offset;
    calculateInternals();
  }

  /**
   * Constructs a new collision geometry object (partially) representing the given rigid body and
   * located at the body's position.
   *
   * @param rigidBody the rigid body
   */
  public CollisionGeometry(RigidBody rigidBody) {
    this.rigidBody = rigidBody;
    offset = new Matrix4();
    calculateInternals();
  }

  /**
   * Calculates and returns the volume of this collision geometry object.
   *
   * @return the space enclosed by this collision geometry
   */
  public abstract double getVolume();

  /**
   * Calculates and returns the current center of the collision geometry object. As the object's
   * position is relative to the associated rigid body, this position will change every time the
   * body changes its own position.
   *
   * @return the object's current center
   */
  public Vector3D getCenter() {
    Vector3D center = new Vector3D(rigidBody.getPosition());
    center.add(new Vector3D(offset.getD(), offset.getH(), offset.getL()));
    return center;
  }

  /**
   * Sets the center of the geometry object by component.
   *
   * @param x the new x coordinate of the center
   * @param y the new y coordinate of the center
   * @param z the new z coordinate of the center
   */
  public void setCenter(double x, double y, double z) {
    offset.setD(x - rigidBody.getPosition().getX());
    offset.setH(y - rigidBody.getPosition().getY());
    offset.setL(z - rigidBody.getPosition().getZ());
  }

  /**
   * Returns the rigid body which is (partially) represented by this geometry.
   *
   * @return {@link #rigidBody}
   */
  public RigidBody getRigidBody() {
    return rigidBody;
  }

  /**
   * Sets the provided rigid body as the body represented by this geometry.
   *
   * @param rigidBody the rigid body represented by this geometry
   */
  public void setRigidBody(RigidBody rigidBody) {
    this.rigidBody = rigidBody;
  }

  /**
   * Returns the translation and rotation of this geometry in relation to the rigid body's position.
   *
   * @return the offset
   */
  public Matrix4 getOffset() {
    return offset;
  }

  /**
   * Sets the primitive's offset from its rigid body's position to the given value.
   *
   * @param offset the primitive's offset
   */
  public void setOffset(Matrix4 offset) {
    this.offset = offset;
  }

  /**
   * Returns the resultant transform of the primitive geometry, calculated from the combined offset
   * of the primitive and the transform (orientation + position) of the rigid body to which it is
   * attached.
   *
   * @return {@link #transform}
   */
  public Matrix4 getTransform() {
    return transform;
  }

  /**
   * Returns any of the axis vectors from the {@link #transform} matrix. Providing an invalid index
   * will result in a {@code NullPointerException}.
   *
   * @param index the index of the vector, the valid range is 0 to 3
   * @return the axis vector
   */
  public Vector3D getAxis(int index) {
    return transform.getAxisVector(index);
  }

  /** Calculates the internals for the primitive collision geometry. */
  public void calculateInternals() {
    transform = Matrix4.multiply(rigidBody.getTransformMatrix(), offset);
  }
}
