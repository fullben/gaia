package de.fullben.processing.gaia.rigidbodies.cgens;

import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;
import de.fullben.processing.gaia.rigidbodies.contacts.Contact;

/**
 * {@code Joint} objects link a pair of rigid bodies by generating contacts whenever the two objects
 * stray too far apart.
 *
 * @author Benedikt Full
 */
public class Joint implements ContactGenerator {

  /** The two rigid bodies that are connected by this joint. */
  private RigidBody[] bodies;
  /** The relative location of the connection for each body, given in local coordinates. */
  private Vector3D[] positions;
  /**
   * The maximum displacement at the joint before the joint is considered to be violated. This is
   * normally a small epsilon value. It can be large, however, in which case the joint will behave
   * as if an inelastic cable joined the bodies at their joint locations.
   */
  private double error;

  /**
   * Creates a new joint connecting the two provided bodies at the given points.
   *
   * @param bodyA the first of the two rigid bodies connected by this joint
   * @param bodyB the second of the two rigid bodies connected by this joint
   * @param positionA the relative location of the joint connection on the first rigid body
   * @param positionB the relative location of the joint connection on the second rigid body
   * @param error the maximum displacement at the joint before it is considered to be violated
   */
  public Joint(
      RigidBody bodyA, RigidBody bodyB, Vector3D positionA, Vector3D positionB, double error) {
    bodies = new RigidBody[] {bodyA, bodyB};
    positions = new Vector3D[] {positionA, positionB};
    this.error = error;
  }

  /**
   * Generates the contacts required to restore the joint if it has been violated.
   *
   * @param contact the contact object which will be updated with the new contact data
   * @param limit the maximum number of contacts that can be written
   * @return the number of contacts created
   */
  @Override
  public int addContact(Contact contact, int limit) {
    // Calculate the position of each connection point in world coordinates
    Vector3D posWorldA = bodies[0].getPointInWorldSpace(positions[0]);
    Vector3D posWorldB = bodies[1].getPointInWorldSpace(positions[1]);
    // Calculate the length of the joint
    Vector3D aToB = Vector3D.subtract(posWorldB, posWorldA);
    Vector3D normal = new Vector3D(aToB);
    normal.normalize();
    double length = aToB.magnitude();
    // Check if it is violated
    if (Math.abs(length) > error) {
      contact.setContactNormal(normal);
      Vector3D contactPoint = Vector3D.add(posWorldA, posWorldB);
      contactPoint.multiply(0.5);
      contact.setContactPoint(contactPoint);
      contact.setPenetration(length - error);
      contact.setBodyData(bodies[0], bodies[1], 1.0, 0);
      return 1;
    }
    return 0;
  }
}
