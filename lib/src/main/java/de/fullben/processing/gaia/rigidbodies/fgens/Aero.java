package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Matrix3;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A force generator capable of applying an aerodynamic force to a rigid body.
 *
 * @author Benedikt Full
 * @see RigidBody
 */
public class Aero implements ForceGenerator {

  /** The aerodynamic tensor for the surface in body space. */
  private final Matrix3 tensor;
  /** The relative position of the aerodynamic surface in body coordinates. */
  private final Vector3D position;
  /**
   * A vector containing the wind speed of the environment. This is easier than managing a separate
   * wind speed vector per generator and having to update it manually as the wind changes.
   */
  private final Vector3D windSpeed;

  /**
   * Constructs a new aerodynamic force generator with the provided parameters.
   *
   * @param tensor the aerodynamic tensor for the surface in body space
   * @param position the relative position of the aerodynamic surface in body coordinates
   * @param windSpeed the wind speed of the environment
   */
  public Aero(Matrix3 tensor, Vector3D position, Vector3D windSpeed) {
    this.tensor = tensor;
    this.position = position;
    this.windSpeed = windSpeed;
  }

  /**
   * Applies the aerodynamic force to the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  @Override
  public void updateForce(RigidBody body, double duration) {
    updateForceFromTensor(body, duration, tensor);
  }

  /**
   * Uses an explicit tensor matrix to update the force on the given rigid body. This is exactly the
   * same as for {@link #updateForce(RigidBody, double)}, except that it takes an explicit tensor.
   * Applies the aerodynamic force th the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   * @param tensor the aerodynamic tensor in body space
   */
  protected void updateForceFromTensor(RigidBody body, double duration, Matrix3 tensor) {
    // Calculate total velocity
    Vector3D velocity = new Vector3D(body.getVelocity());
    velocity.add(windSpeed);
    // Calculate the velocity in body coordinates
    Vector3D bodyVelocity = body.getTransformMatrix().transformInverseDirection(velocity);
    // Calculate the force in body coordinates
    Vector3D bodyForce = Matrix3.transform(tensor, bodyVelocity);
    Vector3D force = body.getTransformMatrix().transformDirection(bodyForce);
    // Apply force
    body.addForceAtBodyPoint(force, position);
  }

  /**
   * Sets the aerodynamic tensor for the surface in body space.
   *
   * @return the new aerodynamic tensor
   */
  public Matrix3 getTensor() {
    return tensor;
  }
}
