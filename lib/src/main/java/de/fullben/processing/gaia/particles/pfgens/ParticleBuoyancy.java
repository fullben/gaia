package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A force generator capable of simulating a buoyancy force. Buoyancy is the force that keeps
 * objects afloat in a liquid. It is equal to the weight of water that the object displaces.
 *
 * <p>Buoyancy force generators assume that the force is acting in the up direction. In Processing,
 * this is the negative component of the {@code y} axis.
 *
 * @author Benedikt Full
 */
public class ParticleBuoyancy implements ParticleForceGenerator {

  /**
   * Maximum submersion depth of the object before it generates the maximum buoyancy force. This
   * point is reached whenever the object is fully submerged in the water, thus this value should
   * represent the object's height.
   */
  private double maxDepth;
  /** The volume of the object. */
  private double volume;
  /**
   * The height of the liquid plane above {@code y = 0} (values above will be negative, below will
   * be positive. The liquid plane is parallel to the {@code xz} plane.
   */
  private double liquidHeight;
  /** The density of the liquid. Regular water has a density of 1000 kg per cubic meter. */
  private double liquidDensity;

  /**
   * Constructs a new buoyancy force based on the provided parameters.
   *
   * @param maxDepth normally the object's height
   * @param volume the object's volume
   * @param liquidHeight height of the liquid plane above {@code y = 0}, which may be negative
   * @param liquidDensity density of the liquid
   */
  public ParticleBuoyancy(
      double maxDepth, double volume, double liquidHeight, double liquidDensity) {
    this.maxDepth = maxDepth;
    this.volume = volume;
    this.liquidHeight = liquidHeight;
    this.liquidDensity = liquidDensity;
  }

  /**
   * Applies the force to the provided particle if it is at least partially submerged in the liquid.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    // Calculate the submersion depth
    double depth = particle.getPosition().getY();
    if (depth <= liquidHeight - maxDepth) {
      return; // Object is not in contact with liquid
    }
    Vector3D force = new Vector3D();
    // Object is fully submerged
    if (depth >= liquidHeight + maxDepth) {
      force.setY(-(liquidDensity * volume));
      particle.addForce(force);
      return;
    }
    // Object is partially submerged
    force.setY(liquidDensity * volume * (depth + maxDepth + liquidHeight) / (2 * maxDepth));
    particle.addForce(force);
  }
}
