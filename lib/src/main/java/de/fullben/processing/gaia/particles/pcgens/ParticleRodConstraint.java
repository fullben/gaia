package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * A {@code ParticleRodConstraint} connects a {@link Particle} object to an immovable anchor point
 * in 3D space. The distance between the anchor and the {@code Particle} will always be the same.
 * This is achieved by generating a contact whenever the actual distance between them is not equal
 * to the supposed distance.
 *
 * @author Benedikt Full
 */
public class ParticleRodConstraint extends ParticleConstraint {

  /** The length of the rod. */
  private final double length;

  /**
   * Constructs a new rod, connecting the provided {@code Particle} to the immovable anchor point.
   *
   * @param particle the {@code Particle} connected to the {@code anchor}
   * @param anchor the point to which the {@code particle} is connected
   * @param length the length of the rod
   */
  public ParticleRodConstraint(Particle particle, Vector3D anchor, double length) {
    super(particle, anchor);
    this.length = length;
  }

  /**
   * Fills the provided contact with the contact needed to keep the rod from extending or
   * compressing.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit maximum number of contacts that can be written
   * @return 0 if the rod's actual length is equal to its supposed length, 1 if it's not
   */
  @Override
  public int addContact(ParticleContact contact, int limit) {
    // Get rod length
    double currLength = currentLength();
    // Check if rod is overextending
    if (currLength == length) {
      return 0;
    }
    // Create contact if length is not equal to the supposed value
    contact.setParticleOne(getParticle());
    // Calculate normal
    Vector3D normal = Vector3D.subtract(getAnchor(), getParticle().getPosition());
    normal.normalize();
    // Normal depends on whether the rod is extending or compressing
    if (currLength > length) {
      contact.setContactNormal(normal);
      contact.setPenetration(currLength - length);
    } else {
      normal.invert();
      contact.setContactNormal(normal);
      contact.setPenetration(length - currLength);
    }
    // Rods don't bounce
    contact.setRestitution(0);
    return 1;
  }
}
