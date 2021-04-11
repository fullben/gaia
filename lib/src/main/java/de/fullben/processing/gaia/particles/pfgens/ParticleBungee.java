package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A particle force generator capable of simulating bungee-like springs. These type of springs only
 * produce a pulling force.
 *
 * @author Benedikt Full
 */
public class ParticleBungee implements ParticleForceGenerator {

  /** A value defining the bungee's stiffness. */
  private final double springConstant;
  /** The length of the bungee when it is neither compressed nor extended. */
  private final double restLength;
  /** The other {@code Particle} to which the bungee is attached. */
  private final Particle other;

  /**
   * Constructs a new bungee. A bungee is a spring that only produces a pulling force.
   *
   * @param other the other {@code Particle} the bungee is attached to
   * @param springConstant stiffness of the bungee
   * @param restLength length of the bungee when neither compressed nor extended
   */
  public ParticleBungee(Particle other, double springConstant, double restLength) {
    this.other = other;
    this.springConstant = springConstant;
    this.restLength = restLength;
  }

  /**
   * Applies the force produced by the bungee to the provided particle. This method has no effect
   * whenever the length of the bungee is lower or equal to its rest length.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    // Calculate the vector of the spring
    Vector3D force = new Vector3D(particle.getPosition());
    force.subtract(other.getPosition());
    double magnitude = force.magnitude();
    // Check if bungee is compressed
    if (magnitude <= restLength) {
      return;
    }
    magnitude = springConstant * (restLength - magnitude);
    // Calculate the final force and apply to particle
    force.normalize();
    force.multiply(magnitude);
    particle.addForce(force);
  }
}
