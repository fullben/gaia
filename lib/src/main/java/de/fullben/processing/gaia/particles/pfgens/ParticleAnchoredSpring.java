package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A particle force generator capable of applying a force of a spring, where one end is attached to
 * a set point in 3D space. The position of this anchor may be changed if the need arises.
 *
 * @author Benedikt Full
 */
public class ParticleAnchoredSpring implements ParticleForceGenerator {

  /** A value defining the spring's stiffness. */
  private final double springConstant;
  /** The length of the spring when it is neither compressed nor extended. */
  private final double restLength;
  /** The set point in 3D space to which the spring is attached to. */
  private final Vector3D anchor;

  /**
   * Constructs a new anchored spring utilizing the provided parameters.
   *
   * @param anchor the set point the spring is attached to
   * @param springConstant stiffness of the spring
   * @param restLength length of the spring when neither compressed nor extended
   */
  public ParticleAnchoredSpring(Vector3D anchor, double springConstant, double restLength) {
    this.anchor = anchor;
    this.springConstant = springConstant;
    this.restLength = restLength;
  }

  /**
   * Sets the anchor of the spring to the provided position.
   *
   * @param anchor the new position of the spring's anchor
   */
  public void setAnchor(Vector3D anchor) {
    this.anchor.setValues(anchor);
  }

  /**
   * Sets the position of the anchor of the spring to the provided coordinates.
   *
   * @param x the new position on the x axis
   * @param y the new position on the y axis
   * @param z the new position on the z axis
   */
  public void setAnchor(double x, double y, double z) {
    anchor.setValues(x, y, z);
  }

  /**
   * Applies the force of the anchored spring to the provided particle.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    Vector3D force = new Vector3D(particle.getPosition());
    force.subtract(anchor);
    double magnitude = force.magnitude();
    magnitude = (restLength - magnitude) * springConstant;
    force.normalize();
    force.multiply(magnitude);
    particle.addForce(force);
  }
}
