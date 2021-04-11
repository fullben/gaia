package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * A {@code ParticleRod} links two {@link Particle} objects by enforcing that they always have the
 * same spacing between each other. This is achieved by generating a contact if the actual distance
 * between them is not equal to the supposed distance.
 *
 * @author Benedikt Full
 */
public class ParticleRod extends ParticleLink {

  /** The length of the rod. */
  private final double length;

  /**
   * Constructs a new {@code ParticleRod}, linking the two provided {@link Particle} objects.
   *
   * @param particles the two {@code Particle} objects linked by the rod
   * @param length the length of the rod
   */
  public ParticleRod(Particle[] particles, double length) {
    super(particles);
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
    // Get current rod length
    double currentLength = currentLength();
    // Check for overextension
    if (currentLength == length) {
      return 0;
    }
    // Update contact if overextending
    contact.setParticleOne(getParticleOne());
    contact.setParticleTwo(getParticleTwo());
    // Calculate normal
    Vector3D normal =
        Vector3D.subtract(getParticleTwo().getPosition(), getParticleOne().getPosition());
    normal.normalize();
    // Normal depends on whether the rod is extending or compressing
    if (currentLength > length) {
      contact.setContactNormal(normal);
      contact.setPenetration(currentLength - length);
    } else {
      normal.multiply(-1);
      contact.setContactNormal(normal);
      contact.setPenetration(length - currentLength);
    }
    // Rods don't bounce
    contact.setRestitution(0);
    return 1;
  }
}
