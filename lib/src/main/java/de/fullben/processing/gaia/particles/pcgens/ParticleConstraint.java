package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * {@code ParticleConstraint}s are similar to {@link ParticleLink}s, except that they connect one
 * {@link Particle} to an immovable anchor point in 3D space. Like the links, they use collisions to
 * enforce the constraint.
 *
 * @author Benedikt Full
 */
public abstract class ParticleConstraint implements ParticleContactGenerator {

  /** The {@code Particle} connected by this constraint. */
  private final Particle particle;
  /** The point in 3D space to which the {@link #particle} is anchored. */
  private final Vector3D anchor;

  /**
   * Constructs a new constraint connecting the provided particle to the also provided anchor point.
   *
   * @param particle the {@code Particle} connected by this constraint
   * @param anchor the point to which the {@code particle} is connected
   */
  protected ParticleConstraint(Particle particle, Vector3D anchor) {
    this.particle = particle;
    this.anchor = anchor;
  }

  /**
   * Returns the current length of the constraint.
   *
   * @return the current length
   */
  protected double currentLength() {
    Vector3D relativePos = Vector3D.subtract(particle.getPosition(), anchor);
    return relativePos.magnitude();
  }

  /**
   * Generates the contact to keep this constraint from being violated. Instances of this class can
   * only ever generate one contact. Thus the {@code limit} parameter is assumed to be at least 1.
   * The return value 0 if the link was not overextended, or 1 if a contact was created.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit maximum number of contacts that can be written
   * @return the number of contacts created
   */
  @Override
  public abstract int addContact(ParticleContact contact, int limit);

  /**
   * Returns the {@code Particle} affected by this constraint.
   *
   * @return the affected {@code Particle} object
   */
  public Particle getParticle() {
    return particle;
  }

  /**
   * Returns the position in 3D space to which the {@code Particle} affected by this constraint is
   * connected.
   *
   * @return a copy of the position of the anchor point
   */
  public Vector3D getAnchor() {
    return new Vector3D(anchor);
  }
}
