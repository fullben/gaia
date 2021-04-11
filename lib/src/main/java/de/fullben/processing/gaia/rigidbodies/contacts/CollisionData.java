package de.fullben.processing.gaia.rigidbodies.contacts;

import java.util.ArrayList;

/**
 * A helper class that contains information for the collision detector to use in building its
 * contact data. Objects of this class may also be utilized to store contacts.
 *
 * @author Benedikt Full
 */
public class CollisionData {

  /** Contacts list to write to. */
  private ArrayList<Contact> contacts;
  /** The friction value to write into any collisions. */
  private double friction;
  /** The restitution value to write to any collisions; */
  private double restitution;
  /**
   * The collision tolerance, even objects that are not colliding but are this close to each other
   * should have collisions generated.
   */
  private double tolerance;

  /**
   * Constructs a new {@code CollisionData} object based on the given parameters.
   *
   * @param friction the friction values of any collisions associated with this object
   * @param restitution the restitution value of any collisions associated with this object
   * @param tolerance the collision tolerance
   */
  public CollisionData(double friction, double restitution, double tolerance) {
    contacts = new ArrayList<>();
    this.friction = friction;
    this.restitution = restitution;
    this.tolerance = tolerance;
  }

  /**
   * Adds the provided contact to this objects contact list.
   *
   * @param contact the contact to add
   */
  public void add(Contact contact) {
    contacts.add(contact);
  }

  /**
   * Adds the provided contacts to this objects contact list.
   *
   * @param contacts the contacts to add
   */
  public void add(ArrayList<Contact> contacts) {
    this.contacts.addAll(contacts);
  }

  /** Clears this object's contact list. */
  public void reset() {
    contacts.clear();
  }

  /**
   * Returns the contact stored in this object.
   *
   * @return {@link #contacts}
   */
  public ArrayList<Contact> getContacts() {
    return contacts;
  }

  /**
   * Returns the friction value, which should be utilized by all collisions associated with this
   * object.
   *
   * @return the value of {@link #friction}
   */
  public double getFriction() {
    return friction;
  }

  /**
   * Sets the friction for all collisions associated with this object to the provided value.
   *
   * @param friction the new friction
   */
  public void setFriction(double friction) {
    this.friction = friction;
  }

  /**
   * Returns the restitution value, which should be utilized by all collisions associated with this
   * object.
   *
   * @return the value of {@link #restitution}
   */
  public double getRestitution() {
    return restitution;
  }

  /**
   * Sets the coefficient of restitution for all collisions associated with this object to the
   * provided value.
   *
   * @param restitution the new coefficient of restitution
   */
  public void setRestitution(double restitution) {
    this.restitution = restitution;
  }

  /**
   * Sets the tolerance to the provided value. Objects that are not colliding but are this close to
   * each other should have collisions generated.
   *
   * @param tolerance the new tolerance value
   */
  public void setTolerance(double tolerance) {
    this.tolerance = tolerance;
  }
}
