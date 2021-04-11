package de.fullben.processing.gaia.particles.pfgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;

/**
 * A force generator capable of applying its gravitational force to any {@link Particle} associated
 * with it.
 *
 * @author Benedikt Full
 */
public class ParticleGravity implements ParticleForceGenerator {

  /**
   * The default value for the {@code y} component of the gravitational force pulling any objects
   * affected by it towards the lower end of the Processing coordinate system. The value itself is
   * positive, as the values along the vertical axis of the Processing 3D coordinate system increase
   * when traversing it from top to bottom.
   */
  private static final double DEFAULT_GRAVITY = 9.80665;
  /** The gravitational force as vector. */
  private Vector3D gravity;

  /**
   * Constructs the default gravitational force. This force pulls any affected objects down along
   * the {@code y} axis of the Processing 3D coordinate system.
   *
   * <p>Note: Processing's vertical axis (the {@code y} axis) appears inverted when compared to some
   * other coordinate systems. Thus it has negative values above the origin and positive values
   * below the origin.
   */
  public ParticleGravity() {
    this(0, DEFAULT_GRAVITY, 0);
  }

  /**
   * Constructs a new gravitational force pointing in the direction defined by the provided values.
   *
   * @param x the force component on the x axis of 3D space
   * @param y the force component on the y axis of 3D space
   * @param z the force component on the z axis of 3D space
   */
  public ParticleGravity(double x, double y, double z) {
    gravity = new Vector3D(x, y, z);
  }

  /**
   * Applies the gravitational force to any {@link Particle}s that do have a finite mass.
   *
   * @param particle a {@code Particle} affected by the force
   * @param duration duration of the frame in seconds
   */
  @Override
  public void updateForce(Particle particle, double duration) {
    if (particle.hasFiniteMass()) {
      Vector3D force = new Vector3D(gravity.getX(), gravity.getY(), gravity.getZ());
      force.multiply(particle.getMass());
      particle.addForce(force);
    } else {
      System.err.println("Unable to apply a force to a particle with infinite mass");
    }
  }

  /**
   * Returns a copy of the current gravitational force.
   *
   * @return a copy of {@link #gravity}
   */
  public Vector3D getGravity() {
    return new Vector3D(gravity);
  }
}
