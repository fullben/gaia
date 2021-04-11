package de.fullben.processing.gaia.particles.pcgens;

import de.fullben.processing.gaia.particles.contacts.ParticleContact;

/**
 * {@code ParticleContactGenerator} objects are capable of creating contacts between particles.
 *
 * @author Benedikt Full
 */
public interface ParticleContactGenerator {

  /**
   * Fills the provided contact with the data of the newly generated contact.
   *
   * <p><b>Note:</b> this method's signature makes no sense, {@code limit} and {@code return int}
   * will probably be removed at some point in the near future.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit maximum number of contacts that can be written
   * @return the number of contacts created
   */
  int addContact(ParticleContact contact, int limit);
}
