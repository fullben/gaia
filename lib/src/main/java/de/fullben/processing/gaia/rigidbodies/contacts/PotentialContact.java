package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * Stores a potential contact to check later.
 *
 * @author Benedikt Full
 */
public class PotentialContact {

  /** The first of the two bodies that might be in contact. */
  private final RigidBody bodyA;
  /** The second of the two bodies that might be in contact. */
  private final RigidBody bodyB;

  /**
   * Constructs a new potential contacts involving the two provided rigid bodies.
   *
   * @param bodyA the first of the two bodies involved
   * @param bodyB the second of the two bodies involved
   */
  public PotentialContact(RigidBody bodyA, RigidBody bodyB) {
    this.bodyA = bodyA;
    this.bodyB = bodyB;
  }

  /**
   * Returns whether this potential contact involves the two given bodies.
   *
   * @param bodyA one of the bodies of this potential contact
   * @param bodyB the other of the bodies of this potential contact
   * @return {@code true} if the potential contact is between the two given bodies, {@code false} in
   *     any other case
   */
  public boolean isBetween(RigidBody bodyA, RigidBody bodyB) {
    return bodyA.equals(this.bodyA) && bodyB.equals(this.bodyB)
        || bodyA.equals(this.bodyB) && bodyB.equals(this.bodyA);
  }

  /**
   * Returns the first of the two rigid bodies involved in the contact.
   *
   * @return the first body
   */
  public RigidBody getBodyA() {
    return bodyA;
  }

  /**
   * Returns the second of the two rigid bodies involved in the contact.
   *
   * @return the second body
   */
  public RigidBody getBodyB() {
    return bodyB;
  }
}
