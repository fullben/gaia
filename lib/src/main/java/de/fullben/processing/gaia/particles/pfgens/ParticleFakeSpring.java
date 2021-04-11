package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A particle force generator capable of simulating a stiff spring force. One end of the spring is
 * attached to a fixed point in 3D space.
 *
 * <p>The force generator fakes a stiff spring by determining the position/length of the spring and
 * calculating the force necessary to move the particle attached to the string to the found
 * position. This force is then applied to the particle.
 *
 * @author Benedikt Full
 */
public class ParticleFakeSpring implements ParticleForceGenerator {
  /** The set point in 3D space to which the spring is attached to. */
  private Vector3D anchor;
  /** A value defining the spring's stiffness. */
  private double springConstant;
  /** The damping on the oscillation of the spring. */
  private double damping;

  /**
   * Constructs a new stiff spring which is attached to the provided point in 3D space.
   *
   * @param anchor the set point the spring is attached to
   * @param springConstant stiffness of the spring
   * @param damping the drag experienced by the spring
   */
  public ParticleFakeSpring(Vector3D anchor, double springConstant, double damping) {
    this.anchor = anchor;
    this.springConstant = springConstant;
    this.damping = damping;
  }

  /**
   * Applies the force to the provided {@code particle}. This method has no effect if the {@code
   * particle}'s mass is infinite.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    // Update only when finite mass
    if (particle.hasFiniteMass()) {
      // Calculate relative position of particle to anchor
      Vector3D position = new Vector3D(particle.getPosition());
      position.subtract(anchor);
      // Calculate constants and check that they are within bounds
      double gamma = 0.5 * Math.sqrt(4 * springConstant - damping * damping);
      if (gamma == 0.0) {
        return;
      }
      Vector3D c =
          Vector3D.add(
              Vector3D.multiply(position, (damping / (2.0 * gamma))),
              Vector3D.multiply(particle.getVelocity(), (1.0 / gamma)));
      // Calculate target position
      Vector3D target =
          Vector3D.add(
              Vector3D.multiply(position, Math.cos(gamma * duration)),
              Vector3D.multiply(c, Math.sin(gamma * duration)));
      target.multiply(Math.exp(-0.5f * duration * damping));
      // Calculate the resulting acceleration and thus the force
      Vector3D acc =
          Vector3D.subtract(
              Vector3D.multiply(Vector3D.subtract(target, position), 1.0 / (duration * duration)),
              Vector3D.multiply(particle.getVelocity(), 1.0 / duration));
      acc.multiply(particle.getMass());
      particle.addForce(acc);
    }
  }
}
