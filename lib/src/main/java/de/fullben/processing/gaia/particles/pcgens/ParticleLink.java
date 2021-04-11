package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * {@code ParticleLink} objects can be used to simulate collision-like things, e.g. cables or rods.
 * These constructs use collisions to keep {@code Particle} objects together.
 *
 * @author Benedikt Full
 */
public abstract class ParticleLink implements ParticleContactGenerator {

  /** The pair of {@code Particle}s that are connected by this link. */
  private Particle[] particles;

  /**
   * Constructs a new {@code ParticleLink}.
   *
   * @param particles the two {@code Particle} objects connected by the link
   */
  public ParticleLink(Particle[] particles) {
    setParticles(particles);
  }

  /**
   * Validates that the provided array has the length two and contains two {@code Particle} objects
   * before setting it as {@link #particles}.
   *
   * @param particles the two {@code Particle} objects connected by the link
   */
  private void setParticles(Particle[] particles) {
    if (!(particles.length == 2 && particles[0] != null && particles[1] != null)) {
      throw new IllegalArgumentException("Array containing two particles expected");
    } else {
      this.particles = particles;
    }
  }

  /**
   * Returns the current length of the link.
   *
   * @return the current length
   */
  protected double currentLength() {
    Vector3D relativePos =
        Vector3D.subtract(getParticleOne().getPosition(), getParticleTwo().getPosition());
    return relativePos.magnitude();
  }

  /**
   * Generates the contact to keep this link from being violated. Instances of this class can only
   * ever generate one contact. Thus the {@code limit} parameter is assumed to be at least 1. The
   * return value 0 if the link was not overextended, or 1 if a contact was created.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit maximum number of contacts that can be written
   * @return the number of contacts created
   */
  @Override
  public abstract int addContact(ParticleContact contact, int limit);

  /**
   * Returns the first of the two {@code Particle}s connected by this link.
   *
   * @return the first {@code Particle}
   */
  public Particle getParticleOne() {
    return particles[0];
  }

  /**
   * Returns the second of the two {@code Particle}s connected by this link.
   *
   * @return the second {@code Particle}
   */
  public Particle getParticleTwo() {
    return particles[1];
  }
}
