package de.fullben.processing.gaia.rigidbodies.cgens;


import de.fullben.processing.gaia.rigidbodies.contacts.Contact;

/**
 * Contact generators are capable of creating contacts between two rigid bodies or a rigid body and
 * an immovable piece of geometry.
 *
 * @author Benedikt Full
 */
public interface ContactGenerator {

  /**
   * Fills the provided contact with the data of the newly generated contact.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit the maximum number of contacts that can be written
   * @return the number of contacts created
   */
  int addContact(Contact contact, int limit);
}
