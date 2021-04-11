package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A force generator capable of applying a basic form of drag to a {@link Particle}. Drag is a force
 * that acts on a body depending on its velocity. It acts in the opposite direction to the velocity.
 *
 * @author Benedikt Full
 */
public class ParticleDrag implements ParticleForceGenerator {

  /** The velocity drag coefficient. */
  private final double k1;
  /**
   * The velocity squared drag coefficient. This value may cause significant growth of the force
   * whenever the velocity of the affected {@link Particle} is high.
   */
  private final double k2;

  /**
   * Constructs a new drag force generator with the provided values as drag coefficients.
   *
   * @param k1 velocity drag coefficient
   * @param k2 velocity squared drag coefficient
   */
  public ParticleDrag(double k1, double k2) {
    this.k1 = k1;
    this.k2 = k2;
  }

  /**
   * Applies the drag force generated based on the velocity of the provided particle to the
   * particle.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    Vector3D force = new Vector3D(particle.getVelocity());
    // Calculate the total drag coefficient
    double dragCoefficient = force.magnitude();
    dragCoefficient = k1 * dragCoefficient + k2 * dragCoefficient * dragCoefficient;
    // Calculate final force and apply
    force.normalize();
    force.multiply(-dragCoefficient);
    particle.addForce(force);
  }
}
