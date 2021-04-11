package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A particle force generator capable of simulating a basic spring. This is achieved by calculating
 * the spring's length followed by the calculation of the spring force using Hook's law.
 *
 * <p>Unlike e.g. {@link ParticleGravity} force generators, instances of this class may not be used
 * in conjunction with multiple particles unless these particles are all attached to the same other
 * particle and their springs have the same stiffness and length.
 *
 * @author Benedikt Full
 */
public class ParticleSpring implements ParticleForceGenerator {

  /** A value defining the spring's stiffness. */
  private final double springConstant;
  /** The length of the spring when it is neither compressed nor extended. */
  private final double restLength;
  /** The other {@code Particle} to which the spring is attached. */
  private final Particle other;

  /**
   * Constructs a new spring utilizing the provided values. The generator will assume that the
   * Spring has the provided stiffness and rest length and is attached to the also provided
   * particle.
   *
   * @param other the other {@code Particle} the spring is attached to
   * @param springConstant stiffness of the spring
   * @param restLength length of the spring when neither compressed nor extended
   */
  public ParticleSpring(Particle other, double springConstant, double restLength) {
    this.other = other;
    this.springConstant = springConstant;
    this.restLength = restLength;
  }

  /**
   * Applies the spring force to the provided particle.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    // Calculate vector of spring
    Vector3D force = new Vector3D(particle.getPosition());
    force.subtract(other.getPosition());
    // Calculate magnitude of spring force
    double magnitude = force.magnitude();
    magnitude = magnitude - restLength;
    magnitude *= springConstant;
    // Calculate final force and apply to particle
    force.normalize();
    force.multiply(-magnitude);
    particle.addForce(force);
  }
}
