package de.fullben.processing.gaia.rigidbodies.fgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A force generator to apply a buoyancy force to a rigid body. Buoyancy is the force that keeps
 * objects afloat in a liquid. It is equal to the weight of water that the object displaces.
 *
 * @author Benedikt Full
 * @see RigidBody
 */
public class Buoyancy implements ForceGenerator {

  /** The center of buoyancy of the rigid body, in body coordinates. */
  private final Vector3D centerOfBuoyancy;
  /**
   * Maximum submersion depth of the object before it generates the maximum buoyancy force. This
   * point is reached whenever the object is fully submerged in the water, thus this value should
   * represent the object's height.
   */
  private final double maxDepth;
  /** The volume of the rigid body. */
  private final double volume;
  /**
   * The height of the liquid plane above {@code y = 0} (values above will be negative, below will
   * be positive. The plane will be parallel to the {@code xz} plane.
   */
  private final double liquidHeight;
  /** The density of the liquid. Pure water has a density of 1000 kg per cubic meter. */
  private double liquidDensity;

  /**
   * Constructs a new buoyancy force generator utilizing the provided parameters and assuming that
   * the simulated liquid has the same density as water, 1000 kg per cubic meter.
   *
   * @param centerOfBuoyancy the center of buoyancy of the rigid body in body coordinates
   * @param maxDepth the maximum submersion depth of the body
   * @param volume the volume of the rigid body
   * @param liquidHeight the height of the liquid plane above the xz plane
   */
  public Buoyancy(Vector3D centerOfBuoyancy, double maxDepth, double volume, double liquidHeight) {
    this.centerOfBuoyancy = centerOfBuoyancy;
    this.maxDepth = maxDepth;
    this.volume = volume;
    this.liquidHeight = liquidHeight;
    liquidDensity = 1000.0;
  }

  /**
   * Applies the force to the given rigid body.
   *
   * @param body the rigid body to which this method applies a force
   * @param duration the duration for which to apply the force in seconds
   */
  @Override
  public void updateForce(RigidBody body, double duration) {
    // Calculate submersion depth
    Vector3D pointInWorld = body.getPointInWorldSpace(centerOfBuoyancy);
    double depth = pointInWorld.getY();
    // Check if we are out of the water
    if (depth <= liquidHeight - maxDepth) {
      return;
    }
    Vector3D force = new Vector3D();
    // Check if we are at maximum depth
    if (depth >= liquidHeight + maxDepth) {
      force.setY(-(liquidDensity * volume));
      body.addForceAtBodyPoint(force, centerOfBuoyancy);
      return;
    }
    // We are partly submerged
    force.setY(-(liquidDensity * volume * (depth + maxDepth + liquidHeight) / (2 * maxDepth)));
    body.addForceAtBodyPoint(force, centerOfBuoyancy);
  }

  /**
   * Returns the height of the liquid plane above {@code y = 0}. Values above will be negative,
   * below will be positive.
   *
   * @return the liquid level
   */
  public double getLiquidHeight() {
    return liquidHeight;
  }

  /**
   * Sets the liquid's density to the provided value.
   *
   * @param liquidDensity new density of the liquid
   */
  public void setLiquidDensity(double liquidDensity) {
    this.liquidDensity = liquidDensity;
  }
}
