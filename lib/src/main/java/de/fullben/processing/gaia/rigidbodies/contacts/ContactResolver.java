package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Vector3D;
import java.util.List;

/**
 * The contact resolution routine. One resolver instance can be shared for the entire simulation, as
 * long as all components require roughly the same parameters each time.
 *
 * <p>
 *
 * <h3>Resolution algorithm</h3>
 *
 * The algorithm employed by the resolver is an iterative satisfaction algorithm; it loops through
 * each contact and tries to resolve it. Each contact is resolved locally, which may in turn put
 * other contacts in a worse position. The algorithm then revisits other contacts and repeats the
 * process up to a specified iteration limit. It can be proved that given enough iterations, the
 * simulation will get to the correct result. As with all approaches, numerical stability can cause
 * problems that make a correct resolution impossible.
 *
 * <p>
 *
 * <h3>Strengths</h3>
 *
 * Many global algorithms are unstable under high friction, this approach is very robust for high
 * friction and low restitution values.
 *
 * <p>The algorithm produces visually believable behavior. Trade-offs have been made to err on the
 * side of visual realism rather than computational expense or numerical accuracy.
 *
 * <p>
 *
 * <h3>Weaknesses</h3>
 *
 * The algorithm does not cope well with situations with many inter-related contacts: stacked boxes,
 * for example. In this case the simulation may appear to jiggle slightly, which often dislodges a
 * box from the stack, allowing it to collapse.
 *
 * <p>Another issue with the resolution mechanism is that resolving one contact may make another
 * contact move sideways against friction, because each contact is handled independently, this
 * friction is not taken into account. If one object is pushing against another, the pushed object
 * may move across its support without friction, even though friction is set between those bodies.
 *
 * <p>In general, this resolver is not suitable for stacks of bodies, but is perfect for handling
 * impact, explosive and flat resting situations.
 *
 * @author Benedikt Full
 */
public class ContactResolver {

  /** The default value for {@link #velocityIterations}. */
  private static final int DEFAULT_VELOCITY_ITERATIONS = 3;
  /** The default value for {@link #positionIterations}. */
  private static final int DEFAULT_POSITION_ITERATIONS = 10;
  /** The default value for {@link #velocityEpsilon}. */
  private static final double DEFAULT_VELOCITY_EPSILON = 0.001;
  /** The default value for {@link #positionEpsilon}. */
  private static final double DEFAULT_POSITION_EPSILON = 0.001;
  /** The number of iterations to perform when resolving velocity. */
  private int velocityIterations;
  /** The number of iterations to perform when resolving position. */
  private int positionIterations;
  /**
   * To avoid instability, velocities smaller than this value are considered to be zero. Too small
   * and the simulation may be unstable, too large and the bodies may interpenetrate visually.
   */
  private double velocityEpsilon;
  /**
   * To avoid instability, penetrations smaller than this value are considered to be not
   * interpenetrating. Too small and the simulation may be unstable, too large and the bodies may
   * interpenetrate visually.
   */
  private final double positionEpsilon;

  /**
   * Creates a new contact resolver with the given parameter values.
   *
   * @param velocityIterations the number of iterations performed when resolving velocity
   * @param positionIterations the number of iterations performed when resolving positions
   * @param velocityEpsilon the threshold for velocities - values smaller than this are considered
   *     to be zero
   * @param positionEpsilon the threshold for penetration depth - values smaller than this are
   *     considered to be zero
   */
  public ContactResolver(
      int velocityIterations,
      int positionIterations,
      double velocityEpsilon,
      double positionEpsilon) {
    this.velocityIterations = velocityIterations;
    this.positionIterations = positionIterations;
    this.velocityEpsilon = velocityEpsilon;
    this.positionEpsilon = positionEpsilon;
  }

  /**
   * Creates a new contact resolver with the given values. The numbers of iterations for velocity
   * and position resolution are set to the value of {@code iterations}.
   *
   * @param iterations the number of iterations performed when resolving velocity and positions
   *     respectively
   * @param velocityEpsilon the threshold for velocities - values smaller than this are considered
   *     to be zero
   * @param positionEpsilon the threshold for penetration depth - values smaller than this are
   *     considered to be zero
   */
  public ContactResolver(int iterations, double velocityEpsilon, double positionEpsilon) {
    this(iterations, iterations, velocityEpsilon, positionEpsilon);
  }

  /**
   * Creates a new contact resolver with the given values. The numbers of iterations for velocity
   * and position resolution are set to the value of {@code iterations}. The threshold values for
   * velocity and position values are set to their respective default value.
   *
   * @param iterations the number of iterations performed when resolving velocity and positions
   *     respectively
   */
  public ContactResolver(int iterations) {
    this(iterations, iterations, DEFAULT_VELOCITY_EPSILON, DEFAULT_POSITION_EPSILON);
  }

  /**
   * Constructs a new contact resolver by initializing a new object with default values for
   * iteration counts and velocity and position thresholds.
   */
  public ContactResolver() {
    this(
        DEFAULT_VELOCITY_ITERATIONS,
        DEFAULT_POSITION_ITERATIONS,
        DEFAULT_VELOCITY_EPSILON,
        DEFAULT_POSITION_EPSILON);
  }

  /**
   * Resolves a set of contacts for both penetration and velocity. The resolution is performed in
   * three steps:
   *
   * <ol>
   *   <li>Calculate internal data for each contact
   *   <li>Resolve penetration
   *   <li>Resolve velocity
   * </ol>
   *
   * <p><b>Note:</b> Contacts that cannot interact with each other should be passed to separate
   * calls to this method, as the resolution algorithm takes much longer for lots of contacts than
   * it does for the same number of contacts in small sets.
   *
   * @param contacts the contacts to resolve
   * @param duration the duration for which to resolve in seconds
   */
  public void resolveContacts(List<Contact> contacts, double duration) {
    // Early out if there are no contacts
    if (contacts.size() == 0) {
      return;
    }
    // Prepare contacts for processing
    prepareContacts(contacts, duration);
    // Resolve the interpenetration problems with the contacts
    adjustPositions(contacts, duration);
    // Resolve the velocity problems with the contacts
    adjustVelocities(contacts, duration);
  }

  /**
   * Sets up contacts ready for processing. This ensures that their internal data is configured
   * correctly and the correct set of bodies is made available.
   *
   * @param contacts the contacts to prepare
   * @param duration the duration for which to calculate the contact data
   */
  private void prepareContacts(List<Contact> contacts, double duration) {
    for (Contact c : contacts) {
      // Calculate the internal data (inertia, basis, etc.)
      c.calculateInternals(duration);
    }
  }

  /**
   * Resolves the positional issues with the given contacts, using the value of {@link
   * #positionIterations} as the maximum number of resolution iterations.
   *
   * @param contacts the contacts for which to resolve positional issues
   * @param duration the duration for which to resolve in seconds
   */
  private void adjustPositions(List<Contact> contacts, double duration) {
    int i, index, positionIterationsUsed;
    Vector3D[] linearChanges = new Vector3D[] {new Vector3D(), new Vector3D()};
    Vector3D[] angularChanges = new Vector3D[] {new Vector3D(), new Vector3D()};
    double max;
    Vector3D deltaPosition;
    // Iteratively resolve interpenetration in order of severity
    positionIterationsUsed = 0;
    while (positionIterationsUsed < positionIterations) {
      // Find biggest penetration
      max = positionEpsilon;
      index = contacts.size();
      for (i = 0; i < contacts.size(); i++) {
        if (contacts.get(i).getPenetration() > max) {
          max = contacts.get(i).getPenetration();
          index = i;
        }
      }
      if (index == contacts.size()) {
        break;
      }
      // Match the awake state at the contact
      contacts.get(index).matchAwakeState();
      // Resolve the penetration
      contacts.get(index).applyPositionChange(linearChanges, angularChanges, max);
      // Again this action may have changed the penetration of other bodies, so we update contacts
      for (i = 0; i < contacts.size(); i++) {
        // Check each body in the contact
        for (int j = 0; j < 2; j++) {
          final Contact contact = contacts.get(i);
          if (contact.getBodies()[j] == null) {
            continue;
          }
          // Check for a match with each body in the newly resolved contact
          for (int k = 0; k < 2; k++) {
            if (contact.getBodies()[j].equals(contacts.get(index).getBodies()[k])) {
              deltaPosition =
                  Vector3D.add(
                      linearChanges[k],
                      Vector3D.vectorProduct(
                          angularChanges[k], contact.getRelativeContactPositions()[j]));
              // The sign of the change is positive if we are dealing with the second body in a
              // contact and negative otherwise because we are subtracting the resolution
              contact.setPenetration(
                  contact.getPenetration()
                      + deltaPosition.scalarProduct(contact.getContactNormal()) * (j > 0 ? 1 : -1));
            }
          }
        }
      }
      positionIterationsUsed++;
    }
  }

  /**
   * Resolves the velocity issues with the given contacts, using the value of {@link
   * #velocityIterations} as the maximum number of resolution iterations.
   *
   * @param contacts the contacts for which to resolve the velocity issues
   * @param duration the duration for which to resolve in seconds
   */
  private void adjustVelocities(List<Contact> contacts, double duration) {
    Vector3D[] velocityChanges = new Vector3D[] {new Vector3D(), new Vector3D()};
    Vector3D[] rotationChanges = new Vector3D[] {new Vector3D(), new Vector3D()};
    Vector3D deltaVel;
    int velocityIterationsUsed = 0;
    while (velocityIterationsUsed < velocityIterations) {
      double max = velocityEpsilon;
      int index = contacts.size();
      for (int i = 0; i < contacts.size(); i++) {
        if (contacts.get(i).getDesiredDeltaVelocity() > max) {
          max = contacts.get(i).getDesiredDeltaVelocity();
          index = i;
        }
      }
      if (index == contacts.size()) {
        break;
      }
      // Match the awake state at the contact
      contacts.get(index).matchAwakeState();
      // Do the resolution on the contact that came out top
      contacts.get(index).applyVelocityChange(velocityChanges, rotationChanges);
      // With the change in velocity of the two bodies, the update of contact velocities means that
      // some of the relative closing velocities need recomputing
      for (int i = 0; i < contacts.size(); i++) {
        // Check each body in the contact
        for (int j = 0; j < 2; j++) {
          if (contacts.get(i).getBodies()[j] != null) {
            // Check for a match with each body in the newly resolved contact
            for (int k = 0; k < 2; k++) {
              if (contacts.get(i).getBodies()[j].equals(contacts.get(index).getBodies()[k])) {
                deltaVel =
                    Vector3D.add(
                        velocityChanges[k],
                        Vector3D.vectorProduct(
                            rotationChanges[k], contacts.get(i).getRelativeContactPositions()[j]));
                // The sign of the change is negative if we are dealing with the second body in a
                // contact
                contacts
                    .get(i)
                    .getContactVelocity()
                    .add(contacts.get(i).getContactToWorld().transformTranspose(deltaVel));
                contacts.get(i).getContactVelocity().multiply((j > 0 ? -1 : 1));
                contacts.get(i).calculateDesiredDeltaVelocity(duration);
              }
            }
          }
        }
      }
      velocityIterationsUsed++;
    }
  }

  /**
   * Returns the current velocity resolution iteration limit.
   *
   * @return the value of {@link #velocityIterations}
   */
  public int getVelocityIterations() {
    return velocityIterations;
  }

  /**
   * Sets the velocity iteration count to the given value if it is larger than zero.
   *
   * @param velocityIterations a number larger than zero
   * @return true if the given number was valid
   */
  public boolean setVelocityIterations(int velocityIterations) {
    if (velocityIterations > 0) {
      this.velocityIterations = velocityIterations;
      return true;
    }
    return false;
  }

  /**
   * Returns the velocity limit of the resolver. The resolver considers any velocity values below
   * this limit to be zero.
   *
   * @return {@link #velocityEpsilon}
   */
  public double getVelocityEpsilon() {
    return velocityEpsilon;
  }

  /**
   * Sets the value of {@link #velocityEpsilon} to the given value.
   *
   * @param velocityEpsilon the new velocity epsilon
   */
  public void setVelocityEpsilon(double velocityEpsilon) {
    this.velocityEpsilon = velocityEpsilon;
  }

  /**
   * Returns the current position resolution iteration limit.
   *
   * @return the value of {@link #positionIterations}
   */
  public int getPositionIterations() {
    return positionIterations;
  }

  /**
   * Sets the position iteration count to the given value if it is larger than zero.
   *
   * @param positionIterations a number larger than zero
   * @return true if the given number was valid
   */
  public boolean setPositionIterations(int positionIterations) {
    if (positionIterations > 0) {
      this.positionIterations = positionIterations;
      return true;
    }
    return false;
  }

  /**
   * Returns the position limit of the resolver. The resolver considers any penetration values below
   * this limit to be zero.
   *
   * @return the value of {@link #positionEpsilon}
   */
  public double getPositionEpsilon() {
    return positionEpsilon;
  }

  /**
   * Sets the value of {@link #positionEpsilon} to the given value.
   *
   * @param positionEpsilon the new position epsilon
   */
  public void setPositionEpsilon(double positionEpsilon) {}
}
