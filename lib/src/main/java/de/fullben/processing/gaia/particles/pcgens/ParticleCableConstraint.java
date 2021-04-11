package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * {@code ParticleCableConstraint} objects connect one {@code Particle} to an immovable anchor point
 * in 3D space. The {@code Particle} and its anchor are connected by a cable. This means that the
 * {@code Particle} may move freely as long as the distance between it and the anchor point does not
 * exceed the cable length.
 *
 * @author Benedikt Full
 */
public class ParticleCableConstraint extends ParticleConstraint {

  /** The maximum length of the cable. */
  private final double maxLength;
  /** The bounciness of the cable. */
  private final double restitution;

  /**
   * Constructs a new cable connecting the provided particle to the anchor.
   *
   * @param particle the {@code Particle} connected by this constraint
   * @param anchor the point to which the {@code particle} is connected
   * @param maxLength the maximum length of the cable
   * @param restitution the bounciness of the cable
   */
  public ParticleCableConstraint(
      Particle particle, Vector3D anchor, double maxLength, double restitution) {
    super(particle, anchor);
    this.maxLength = maxLength;
    this.restitution = restitution;
  }

  /**
   * Fills the provided contact object with the contact data required to keep the cable from
   * overextending.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit maximum number of contacts that can be written
   * @return 0 if the cable's length does not exceed its maximum length, 1 if it does
   */
  @Override
  public int addContact(ParticleContact contact, int limit) {
    // Find length of cable
    double length = currentLength();
    // Check if overextended
    if (length < maxLength) {
      return 0;
    }
    // If overextended, create contact
    contact.setParticleOne(getParticle());
    // Calculate normal
    Vector3D normal = Vector3D.subtract(getAnchor(), getParticle().getPosition());
    normal.normalize();
    contact.setContactNormal(normal);
    contact.setPenetration(length - maxLength);
    contact.setRestitution(restitution);
    return 1;
  }
}
