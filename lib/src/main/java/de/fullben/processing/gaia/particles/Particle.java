package de.fullben.processing.gaia.particles;

import de.fullben.processing.gaia.math.Vector3D;

/**
 * Instances of this class are simple point mass objects. A {@code Particle} has a position,
 * velocity and acceleration but no orientation.
 *
 * @author Benedikt Full
 */
public class Particle {

  /** The {@code Particle}'s linear position in world space. */
  private final Vector3D position;
  /** The {@code Particle}'s linear velocity in world space. */
  private final Vector3D velocity;
  /**
   * The {@code Particle}'s acceleration. The field can be used to set acceleration due to gravity
   * or any other constant acceleration.
   */
  private final Vector3D acceleration;
  /** Holds the forces applying to the {@code Particle} over the time of one integration step. */
  private final Vector3D forceAccumulator;
  /**
   * The amount of drag applied to linear motion. Damping is required to remove energy added through
   * numerical instability in the integrator.
   */
  private double damping;
  /**
   * The inverse mass of the {@code Particle}. This value is stored instead of the actual mass, as
   * its integration is simpler and because in real-time simulation it is more useful to have
   * objects with infinite mass than zero mass.
   */
  private double inverseMass;

  /**
   * Constructs a new {@code Particle} which will be located at the origin and does not posses any
   * initial velocity or acceleration.
   */
  public Particle() {
    position = new Vector3D();
    velocity = new Vector3D();
    acceleration = new Vector3D();
    forceAccumulator = new Vector3D();
    inverseMass = 1;
    damping = 0.999;
  }

  /**
   * Integrates the {@code Particle} forward in time by the given amount. This function uses a
   * Newton-Euler integration method, which is a linear approximation of the correct integral. For
   * this reason it may be inaccurate in some cases.
   *
   * @param duration time interval over which to update in seconds
   */
  public void integrate(double duration) {
    if (duration > 0.0) {
      if (inverseMass <= 0.0) {
        // Infinite mass
        System.err.println("Unable to update particle with infinite mass");
        return;
      }
      position.addScaledVector(velocity, duration);
      Vector3D resultingAcc = new Vector3D(acceleration);
      resultingAcc.addScaledVector(forceAccumulator, inverseMass);
      velocity.addScaledVector(resultingAcc, duration);
      Vector3D dampingAmount =
          Vector3D.multiply(
              Vector3D.subtract(velocity, Vector3D.multiply(velocity, damping)), duration);
      velocity.subtract(dampingAmount);
      clearAccumulator();
    } else {
      System.err.println("Unable to update particle for zero or negative frame duration");
    }
  }

  /** Turns the {@link #forceAccumulator} into a zero vector. */
  public void clearAccumulator() {
    forceAccumulator.clear();
  }

  /**
   * Returns the inverse mass of the {@code Particle}.
   *
   * @return value of {@link #inverseMass}
   */
  public double getInverseMass() {
    return inverseMass;
  }

  /**
   * Sets the value of the {@link #inverseMass} field to the provided value.
   *
   * @param inverseMass the inverse mass of the {@code Particle}
   */
  public void setInverseMass(double inverseMass) {
    this.inverseMass = inverseMass;
  }

  /**
   * Returns the mass of the {@code Particle}. If the mass is infinite, this method will return
   * {@link Double#MAX_VALUE}.
   *
   * @return the {@code Particle}'s mass
   */
  public double getMass() {
    if (inverseMass == 0.0) {
      return Double.MAX_VALUE;
    } else {
      return 1 / inverseMass;
    }
  }

  /**
   * Sets the value of the {@link #inverseMass} field to one divided by the provided {@code mass}.
   *
   * @param mass the non-zero mass of the {@code Particle}
   * @throws IllegalArgumentException if the given mass value is zero
   */
  public void setMass(double mass) {
    if (mass == 0.0) {
      throw new IllegalArgumentException("Zero mass is not allowed");
    }
    inverseMass = 1 / mass;
  }

  /**
   * Returns true if the mass of the {@code Particle} is finite.
   *
   * <p>This is the case as long as the value of {@link #inverseMass} is larger or equal to zero.
   *
   * @return true if mass is finite
   */
  public boolean hasFiniteMass() {
    return inverseMass >= 0.0;
  }

  /**
   * Sets the damping value of this {@code Particle} to the provided value. Damping is required to
   * remove energy added through numerical instability in the integrator.
   *
   * @param damping in most cases a value close to 1
   */
  public void setDamping(double damping) {
    this.damping = damping;
  }

  /**
   * Returns a copy of the position vector of this {@code Particle}.
   *
   * @return a copy of {@link #position}
   */
  public Vector3D getPosition() {
    return new Vector3D(position);
  }

  /**
   * Sets the values of the provided vector as the {@code Particle}'s new position.
   *
   * @param position the new position
   */
  public void setPosition(Vector3D position) {
    this.position.setValues(position);
  }

  /**
   * Sets the values of the position vector to the provided values.
   *
   * @param x position on the x axis
   * @param y position on the y axis
   * @param z position on the z axis
   */
  public void setPosition(double x, double y, double z) {
    position.setValues(x, y, z);
  }

  /**
   * Returns a copy of the velocity vector of this {@code Particle}.
   *
   * @return a copy of {@link #velocity}
   */
  public Vector3D getVelocity() {
    return new Vector3D(velocity);
  }

  /**
   * Sets the values of the provided vector as the {@code Particle}'s velocity.
   *
   * @param velocity the new velocity
   */
  public void setVelocity(Vector3D velocity) {
    this.velocity.setValues(velocity);
  }

  /**
   * Sets the values of the velocity vector to the provided values.
   *
   * @param x velocity on the x axis
   * @param y velocity on the y axis
   * @param z velocity on the z axis
   */
  public void setVelocity(double x, double y, double z) {
    velocity.setValues(x, y, z);
  }

  /**
   * Returns a copy of the acceleration vector of this {@code Particle}.
   *
   * @return a copy of {@link #acceleration}
   */
  public Vector3D getAcceleration() {
    return new Vector3D(acceleration);
  }

  /**
   * Sets the values provided vector as the {@code Particle}'s new acceleration.
   *
   * @param acceleration the new acceleration
   */
  public void setAcceleration(Vector3D acceleration) {
    this.acceleration.setValues(acceleration);
  }

  /**
   * Sets the values of the acceleration vector to the provided values.
   *
   * @param x acceleration on the x axis
   * @param y acceleration on the x axis
   * @param z acceleration on the z axis
   */
  public void setAcceleration(double x, double y, double z) {
    acceleration.setValues(x, y, z);
  }

  /**
   * Adds the provided vector to the vector accumulating all the forces acting on the {@code
   * Particle}.
   *
   * @param force a force applying to the {@code Particle}
   */
  public void addForce(Vector3D force) {
    forceAccumulator.add(force);
  }
}
