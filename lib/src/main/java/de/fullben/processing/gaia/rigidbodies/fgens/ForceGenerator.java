package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * {@code ForceGenerator} objects can be asked to add a force to a {@link RigidBody}.
 *
 * @author Benedikt Full
 */
public interface ForceGenerator {

  /**
   * Calculates and updates a force applied to the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  void updateForce(RigidBody body, double duration);
}
