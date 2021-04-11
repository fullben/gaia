package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.particles.Particle;
import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * {@code ParticleCable} objects link a pair of {@code Particle}s by generating a contact if the two
 * objects stray too far apart.
 *
 * @author Benedikt Full
 */
public class ParticleCable extends ParticleLink {

  /** The maximum length of the cable. */
  private final double maxLength;
  /** The bounciness of the cable. */
  private final double restitution;

  /**
   * Constructs a new {@code ParticleCable} connecting the two provided particles.
   *
   * @param particles the two {@code Particle} objects connected by the cable
   * @param maxLength the maximum length of the cable
   * @param restitution the bounciness of the cable
   */
  public ParticleCable(Particle[] particles, double maxLength, double restitution) {
    super(particles);
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
    // Find the current cable length
    double length = currentLength();
    // Check for overextension
    if (length < maxLength) {
      return 0;
    }
    // Fill contact object with necessary data
    contact.setParticleOne(getParticleOne());
    contact.setParticleTwo(getParticleTwo());
    Vector3D normal =
        Vector3D.subtract(getParticleTwo().getPosition(), getParticleOne().getPosition());
    normal.normalize();
    contact.setContactNormal(normal);
    contact.setPenetration(length - maxLength);
    contact.setRestitution(restitution);
    return 1;
  }
}
