package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.particles.Particle;

/**
 * {@code ParticleForceGenerator} objects can be used to apply a current force to a {@link
 * Particle}.
 *
 * @author Benedikt Full
 */
public interface ParticleForceGenerator {

  /**
   * The implementation of this method should apply a force to the provided {@link Particle} object.
   * If the strength of the force depends on the duration for which it has to be applied, the {@code
   * duration} parameter may be used for calculating the final force value.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  void updateForce(Particle particle, double duration);
}
