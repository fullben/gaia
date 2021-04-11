package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A force generator that applies a gravitational force. One instance can be used for multiple
 * bodies.
 *
 * @author Benedikt Full
 */
public class Gravity implements ForceGenerator {

  /**
   * The default value for the {@code y} component of the gravitational force pulling any objects
   * affected by it towards the lower end of the {@code Processing} coordinate system. The value
   * itself is positive, as the values along the vertical axis of the {@code Processing} 3D
   * coordinate system increase when traversing it from top to bottom.
   */
  private static final double DEFAULT_GRAVITY = 9.80665;
  /** The acceleration due to gravity. */
  private final Vector3D gravity;

  /**
   * Constructs a new gravity generator, utilizing the default gravitational pull. This force will
   * pull affected objects down along the {@code y} axis of the {@code Processing} coordinate
   * system.
   */
  public Gravity() {
    this(0.0, DEFAULT_GRAVITY, 0.0);
  }

  /**
   * Constructs a new gravitational force pointing in the direction defined by the provided values.
   *
   * @param x the force component on the x axis of 3D space
   * @param y the force component on the y axis of 3D space
   * @param z the force component on the z axis of 3D space
   */
  public Gravity(double x, double y, double z) {
    this.gravity = new Vector3D(x, y, z);
  }

  /**
   * Applies the gravitational force to the provided body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  @Override
  public void updateForce(RigidBody body, double duration) {
    // Verify that the body does not have infinite mass
    if (!body.hasFiniteMass()) {
      return;
    }
    // Apply mass-scaled force
    body.addForce(Vector3D.multiply(gravity, body.getMass()));
  }

  /**
   * Returns the current gravitational force.
   *
   * @return {@link #gravity}
   */
  public Vector3D getGravity() {
    return gravity;
  }

  /**
   * Sets the gravitational force by component.
   *
   * @param x the force component on the x axis of 3D space
   * @param y the force component on the y axis of 3D space
   * @param z the force component on the z axis of 3D space
   */
  public void setGravity(double x, double y, double z) {
    gravity.setValues(x, y, z);
  }
}
