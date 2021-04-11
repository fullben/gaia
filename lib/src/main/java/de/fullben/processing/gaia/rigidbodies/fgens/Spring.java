package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A force generator capable of simulating a spring connecting two rigid bodies.
 *
 * @author Benedikt Full
 */
public class Spring implements ForceGenerator {

  /** The point of connection of the spring in local coordinates. */
  private final Vector3D connectionPoint;
  /** The point of connection of the spring to the other body in that body's local coordinates. */
  private final Vector3D otherConnectionPoint;
  /** The body at the other end of the spring. */
  private final RigidBody other;
  /** A value defining the spring's stiffness. */
  private final double springConstant;
  /** The length of the spring when it is neither compressed nor extended. */
  private final double restLength;

  /**
   * Creates a new {@code Spring} based on the provided parameters
   *
   * @param localConnectionPoint connection point on the body in local coordinates
   * @param otherConnectionPoint connection point on the other body in that body's local coordinates
   * @param other the other body the spring is connected to
   * @param springConstant the spring's stiffness
   * @param restLength the length of the spring when it is neither compressed nor extended
   */
  public Spring(
      Vector3D localConnectionPoint,
      Vector3D otherConnectionPoint,
      RigidBody other,
      double springConstant,
      double restLength) {
    connectionPoint = localConnectionPoint;
    this.otherConnectionPoint = otherConnectionPoint;
    this.other = other;
    this.springConstant = springConstant;
    this.restLength = restLength;
  }

  /**
   * Applies the spring force to the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  @Override
  public void updateForce(RigidBody body, double duration) {
    // Calculate the two ends in world space
    Vector3D lws = body.getPointInWorldSpace(connectionPoint);
    Vector3D ows = other.getPointInWorldSpace(otherConnectionPoint);
    // Calculate the vector of the spring
    Vector3D force = Vector3D.subtract(lws, ows);
    // Calculate the magnitude of the force
    double magnitude = force.magnitude();
    magnitude = magnitude - restLength;
    magnitude *= springConstant;
    // Calculate final force and apply
    force.normalize();
    force.multiply(-magnitude);
    body.addForceAtPoint(force, lws);
  }
}
