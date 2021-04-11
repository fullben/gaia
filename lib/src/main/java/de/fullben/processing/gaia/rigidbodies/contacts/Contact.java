package de.fullben.processing.gaia.rigidbodies.contacts;

import de.fullben.processing.gaia.math.Matrix3;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.RigidBody;

/**
 * A {@code Contact} represents two bodies in contact. Resolving a contact removes the bodies'
 * interpenetration and applies sufficient impulse to keep them apart. Colliding bodies may also
 * rebound. Contacts can be used to represent positional joints, by making the contact constraint
 * keep the bodies in their correct orientation.
 *
 * @author Benedikt Full
 */
public class Contact {

  private static final double VELOCITY_LIMIT = 0.25;
  private static final double ANGULAR_LIMIT = 0.2;
  /**
   * The two rigid bodies involved in the contact. If the contact occurs between a rigid body and an
   * immovable piece of world geometry, the second entry in the array will remain empty.
   */
  private final RigidBody[] bodies;
  /** The friction of the collision. */
  private double friction;
  /**
   * Coefficient of restitution, ratio of the final to initial relative velocity between two objects
   * after they collide (bounciness).
   */
  private double restitution;
  /** The position of the contact in world coordinates. */
  private final Vector3D contactPoint;
  /** The direction of the contact in world coordinates. */
  private final Vector3D contactNormal;
  /**
   * The depth of penetration at the contact point. If both bodies are specified, then the contact
   * point should be midway between the interpenetration points.
   */
  private double penetration;
  /**
   * A transform matrix that converts coordinates in the contact's frame of reference to world
   * coordinates. The columns of this matrix form an orthonormal set of vectors.
   */
  private final Matrix3 contactToWorld;
  /**
   * The closing velocity at the point of contact. This is set when the when the {@link
   * #calculateInternals(double)} method is called.
   */
  private Vector3D contactVelocity;
  /** The required change in velocity for this contact to be resolved. */
  private double desiredDeltaVelocity;
  /**
   * The world space position of the contact point relative to the center of the body. This is set
   * when the {@link #calculateInternals(double)} method is called.
   */
  private final Vector3D[] relativeContactPositions;

  /**
   * Constructs a new and empty contact. Use the {@code Contact}'s setter methods for adding the
   * contact data.
   */
  public Contact() {
    bodies = new RigidBody[2];
    friction = 0;
    restitution = 0;
    contactPoint = new Vector3D();
    contactNormal = new Vector3D();
    penetration = 0;
    contactToWorld = new Matrix3();
    contactVelocity = new Vector3D();
    desiredDeltaVelocity = 0;
    relativeContactPositions = new Vector3D[2];
  }

  /**
   * Sets the data that doesn't normally depend on the position of the contact, i.e. the bodies and
   * their material properties.
   *
   * <p>If the contact represents a collision between a rigid body and some other type of object,
   * simply provide {@code null} as {@code bodyB}.
   *
   * @param bodyA the first of possibly two rigid bodies involved in the contact
   * @param bodyB the second body involved or {@code null}
   * @param friction the friction
   * @param restitution the restitution
   */
  public void setBodyData(RigidBody bodyA, RigidBody bodyB, double friction, double restitution) {
    bodies[0] = bodyA;
    bodies[1] = bodyB;
    this.friction = friction;
    this.restitution = restitution;
  }

  /**
   * Calculates internal data from state data. This is called before the resolution algorithm tries
   * to do any resolution. It should never need to be called manually.
   *
   * @param duration the duration for which to calculate the data in seconds
   */
  protected void calculateInternals(double duration) {
    // If the first rigid body is null, swap with second
    if (bodies[0] == null && bodies[1] != null) {
      swapBodies();
    }
    // Calculate a set of axis at the contact point
    calculateContactBasis();
    // Store the relative position of the contact relative to each body
    relativeContactPositions[0] = Vector3D.subtract(contactPoint, bodies[0].getPosition());
    if (bodies[1] != null) {
      relativeContactPositions[1] = Vector3D.subtract(contactPoint, bodies[1].getPosition());
    }
    // Find the relative velocity of the bodies at the contact point
    contactVelocity = calculateLocalVelocity(0, duration);
    if (bodies[1] != null) {
      contactVelocity.subtract(calculateLocalVelocity(1, duration));
    }
    // Calculate the desired change in velocity for resolution
    calculateDesiredDeltaVelocity(duration);
  }

  /**
   * Reverses the contact. This involves swapping the two rigid bodies and reversing the contact
   * normal. The internal values should then be recalculated using {@link
   * #calculateInternals(double)} (this is not done automatically).
   */
  private void swapBodies() {
    contactNormal.multiply(-1);
    RigidBody tmp = bodies[0];
    bodies[0] = bodies[1];
    bodies[1] = tmp;
  }

  /**
   * Updates the awake state of the rigid bodies involved in this contact. A body will be made awake
   * if it is in contact with a body that is awake.
   */
  protected void matchAwakeState() {
    // If there's only one body and the world involved, the body may not wake up
    if (bodies[1] == null || bodies[0] == null) {
      return;
    }
    boolean body0Awake = bodies[0].isAwake();
    boolean body1Awake = bodies[1].isAwake();
    // Wake up only the sleeping one
    if (body0Awake || body1Awake) {
      if (body0Awake) {
        bodies[1].setIsAwake(true);
      } else {
        bodies[0].setIsAwake(true);
      }
    }
  }

  /**
   * Calculates and sets the internal value for the desired delta velocity.
   *
   * @param duration the duration for which to calculate the velocity in seconds
   */
  protected void calculateDesiredDeltaVelocity(double duration) {
    double velocityFromAcc = 0;
    if (bodies[0].isAwake()) {
      Vector3D prevAcc = new Vector3D(bodies[0].getLastFrameAcceleration());
      prevAcc.multiply(duration);
      velocityFromAcc += prevAcc.scalarProduct(contactNormal);
    }
    if (bodies[1] != null && bodies[1].isAwake()) {
      Vector3D prevAcc = new Vector3D(bodies[1].getLastFrameAcceleration());
      prevAcc.multiply(duration);
      velocityFromAcc -= prevAcc.scalarProduct(contactNormal);
    }
    // Limit restitution if velocity is very slow
    double thisRestitution = restitution;
    if (Math.abs(contactVelocity.getX()) < VELOCITY_LIMIT) {
      thisRestitution = 0.0;
    }
    // Combine the bounce velocity with the removed acceleration velocity
    desiredDeltaVelocity =
        -contactVelocity.getX() - thisRestitution * (contactVelocity.getX() - velocityFromAcc);
  }

  /**
   * Calculates and returns the velocity of the contact point on the body located at the provided
   * index in the {@link #bodies} array.
   *
   * @param bodyIndex the index of the rigid body, either 0 or 1
   * @param duration the duration for which to calculate the velocity in seconds
   * @return the velocity of the body located at {@code bodyIndex} in the {@link #bodies} array or
   *     an exception if the provided index is invalid
   */
  private Vector3D calculateLocalVelocity(int bodyIndex, double duration) {
    RigidBody body = bodies[bodyIndex];
    // Calculate velocity of the contact point
    Vector3D velocity =
        Vector3D.vectorProduct(body.getAngularVelocity(), relativeContactPositions[bodyIndex]);
    velocity.add(body.getVelocity());
    // Turn velocity into contact coordinates
    Vector3D contactVelocity = contactToWorld.transformTranspose(velocity);
    // Calculate amount of velocity that is due to forces without reactions
    Vector3D accVelocity = new Vector3D(body.getLastFrameAcceleration());
    accVelocity.multiply(duration);
    // Calculate the velocity in contact coordinates
    accVelocity = contactToWorld.transformTranspose(accVelocity);
    // Ignore any component of acceleration in the contact normal direction, we only care for
    // planar acceleration
    accVelocity.setX(0.0);
    // Add the planar velocities
    contactVelocity.add(accVelocity);
    return contactVelocity;
  }

  /**
   * Constructs an arbitrary orthonormal basis for the contact, based on the primary friction
   * direction (for anisotropic friction) or a random orientation (for isotropic friction). This
   * basis is stored as a 3 x 3 matrix, where each vector is a column (in other words, the matrix
   * transforms contact space into world space). The x direction is generated from the contact
   * normal and the y and z directions are set so that they are at right angles to it.
   */
  private void calculateContactBasis() {
    Vector3D[] contactTangents = new Vector3D[] {new Vector3D(), new Vector3D()};
    // Check whether the z axis is closer to the x or the y axis
    if (Math.abs(contactNormal.getX()) > Math.abs(contactNormal.getY())) {
      // Scaling factor to ensure the results are normalized
      double s =
          1.0
              / Math.sqrt(
                  contactNormal.getZ() * contactNormal.getZ()
                      + contactNormal.getX() * contactNormal.getX());
      // The new x axis is at right angles to the world y axis
      contactTangents[0].setValues(contactNormal.getZ() * s, 0.0, -contactNormal.getX() * s);
      // The new y axis is at right angles to the new x and z axis
      contactTangents[1].setX(contactNormal.getY() * contactTangents[0].getX());
      contactTangents[1].setY(
          contactNormal.getZ() * contactTangents[0].getX()
              - contactNormal.getX() * contactTangents[0].getZ());
      contactTangents[1].setZ(-contactNormal.getY() * contactTangents[0].getX());
    } else {
      // Scaling factor to ensure the results are normalized
      double s =
          1.0
              / Math.sqrt(
                  contactNormal.getZ() * contactNormal.getZ()
                      + contactNormal.getY() * contactNormal.getY());
      // The new x axis is at right angles to the world x axis
      contactTangents[0].setValues(0.0, -contactNormal.getZ() * s, contactNormal.getY() * s);
      // The new y axis is at right angles to the new x and z axis
      contactTangents[1].setX(
          contactNormal.getY() * contactTangents[0].getZ()
              - contactNormal.getZ() * contactTangents[0].getY());
      contactTangents[1].setY(-contactNormal.getX() * contactTangents[0].getZ());
      contactTangents[1].setZ(contactNormal.getX() * contactTangents[0].getY());
    }
    // Set contact to world matrix to calculated vectors
    contactToWorld.setComponents(contactNormal, contactTangents[0], contactTangents[1]);
  }

  /**
   * Performs an inertia weighted impulse based resolution of this contact alone.
   *
   * <p><b>Note:</b> The provided arrays are supposed to contain at least one vector each. If the
   * contact has only one rigid body, the provided arrays may also only contain just one vector
   * each. If the contact involves two bodies, each array has to contain two vectors. This method
   * has no effect if these conditions are not honored.
   *
   * @param velocityChanges the velocity changes of the contact's bodies, should contain two vectors
   * @param rotationChanges the rotation changes of the contact's bodies, should contain two vectors
   */
  protected void applyVelocityChange(Vector3D[] velocityChanges, Vector3D[] rotationChanges) {
    // Validating dimensions and content of the two provided arrays
    if (bodies[0] != null
        && (velocityChanges.length < 1
            || velocityChanges[0] == null
            || rotationChanges.length < 1
            || rotationChanges[0] == null)) {
      return;
    } else if (bodies[1] != null
        && (velocityChanges.length < 2
            || velocityChanges[1] == null
            || rotationChanges.length < 2
            || rotationChanges[1] == null)) {
      return;
    }
    // Cache inverse mass and inverse inertia tensors of the two bodies
    Matrix3[] inverseInertiaTensors = new Matrix3[2];
    inverseInertiaTensors[0] = bodies[0].getInverseInertiaTensorWorld();
    if (bodies[1] != null) {
      inverseInertiaTensors[1] = bodies[1].getInverseInertiaTensorWorld();
    }
    // Calculate the impulse for each contact axis
    Vector3D impulseContact;
    if (friction == 0.0) {
      // Use the short format for frictionless contacts
      impulseContact = calculateFrictionlessImpulse(inverseInertiaTensors);
    } else {
      // Otherwise, we may have impulses that aren't in the direction of the contact, so we need the
      // more complex version
      impulseContact = calculateFrictionImpulse(inverseInertiaTensors);
    }
    // Convert impulse to world coordinates
    Vector3D impulse = new Vector3D(impulseContact);
    contactToWorld.transform(impulse);
    // Split impulse into linear and rotational components
    Vector3D impulsiveTorque = Vector3D.vectorProduct(relativeContactPositions[0], impulse);
    inverseInertiaTensors[0].transform(impulsiveTorque);
    rotationChanges[0] = impulsiveTorque;
    velocityChanges[0].clear();
    velocityChanges[0].addScaledVector(impulse, bodies[0].getInverseMass());
    // Apply the changes
    bodies[0].addVelocity(velocityChanges[0]);
    bodies[0].addAngularVelocity(rotationChanges[0]);
    if (bodies[1] != null) {
      // Work out the second body's linear and angular changes
      Vector3D impulsiveTorqueB = Vector3D.vectorProduct(impulse, relativeContactPositions[1]);
      inverseInertiaTensors[1].transform(impulsiveTorqueB);
      rotationChanges[1] = impulsiveTorqueB;
      velocityChanges[1].clear();
      velocityChanges[1].addScaledVector(impulse, -bodies[1].getInverseMass());
      // Apply the changes
      bodies[1].addVelocity(velocityChanges[1]);
      bodies[1].addAngularVelocity(rotationChanges[1]);
    }
  }

  /**
   * Performs an inertia weighted penetration resolution of this contact alone. The calculated
   * linear and angular changes are stored in the given arrays.
   *
   * <p><b>Note:</b> The provided arrays are supposed to contain at least one vector each. If the
   * contact has only one rigid body, the provided arrays may also only contain just one vector
   * each. If the contact involves two bodies, each array has to contain two vectors. This method
   * has no effect if these conditions are not honored.
   *
   * @param linearChanges contains the linear movements of the bodies along the contact normal
   * @param angularChanges contains the angular movements of the bodies
   * @param penetration the contact's penetration depth
   */
  protected void applyPositionChange(
      Vector3D[] linearChanges, Vector3D[] angularChanges, double penetration) {
    // Validating parameters
    if (bodies[0] != null
        && (linearChanges.length < 1
            || linearChanges[0] == null
            || angularChanges.length < 1
            || angularChanges[0] == null)) {
      return;
    } else if (bodies[1] != null
        && (linearChanges.length < 2
            || linearChanges[1] == null
            || angularChanges.length < 2
            || angularChanges[1] == null)) {
      return;
    }
    double[] angularMoves = new double[2];
    double[] linearMoves = new double[2];
    double totalInertia = 0;
    double[] linearInertia = new double[2];
    double[] angularInertia = new double[2];
    int bodyCount = 0;
    if (bodies[0] != null) {
      bodyCount++;
    }
    if (bodies[1] != null) {
      bodyCount++;
    }
    // Work out inertia of each object in the direction of the contact normal, due to angular
    // inertia only
    for (int i = 0; i < bodyCount; i++) {
      Matrix3 inverseInertiaTensor = bodies[i].getInverseInertiaTensorWorld();
      // Use the same procedure as for calculating frictionless velocity change to work out the
      // angular inertia
      Vector3D angularInertiaWorld =
          Vector3D.vectorProduct(relativeContactPositions[i], contactNormal);
      inverseInertiaTensor.transform(angularInertiaWorld);
      angularInertiaWorld.vectorProduct(relativeContactPositions[i]);
      angularInertia[i] = angularInertiaWorld.scalarProduct(contactNormal);
      // The linear component is simply the inverse mass
      linearInertia[i] = bodies[i].getInverseMass();
      // Keep track of the total inertia from all components
      totalInertia += linearInertia[i] + angularInertia[i];
      // Break loop here so that totalInertia value is completely calculated before continuing
    }
    // Loop through the bodies again, calculating and applying the changes
    for (int i = 0; i < bodyCount; i++) {
      // Linear and angular movements required are in proportion to the two inverse inertias
      double sign = (i == 0) ? 1 : -1;
      // double inverseInertia = 1.0 / totalInertia;
      // linearMoves[i] = sign * penetration * linearInertia[i] * inverseInertia;
      // angularMoves[i] = sign * penetration * angularInertia[i] * inverseInertia;
      angularMoves[i] = sign * penetration * (angularInertia[i] / totalInertia);
      linearMoves[i] = sign * penetration * (linearInertia[i] / totalInertia);
      // To avoid angular projections that are too great (when mass is large but inertia tensor is
      // small) limit the angular move
      Vector3D projection = new Vector3D(relativeContactPositions[i]);
      projection.addScaledVector(
          contactNormal, -relativeContactPositions[i].scalarProduct(contactNormal));
      // Use small angle approximation for the sine of the angle
      double maxMagnitude = ANGULAR_LIMIT * projection.magnitude();
      if (angularMoves[i] < -maxMagnitude) {
        double totalMove = angularMoves[i] + linearMoves[i];
        angularMoves[i] = -maxMagnitude;
        linearMoves[i] = totalMove - angularMoves[i];
      } else if (angularMoves[i] > maxMagnitude) {
        double totalMove = angularMoves[i] + linearMoves[i];
        angularMoves[i] = maxMagnitude;
        linearMoves[i] = totalMove - angularMoves[i];
      }
      // We have the linear amount of movement required by turning the rigid body (in
      // angularMoves[i]). We now need to calculate the desired rotation to achieve that.
      if (angularMoves[i] == 0) {
        angularChanges[i].clear();
      } else {
        // Work out the direction we'd like to rotate in
        Vector3D targetAngularDirection =
            Vector3D.vectorProduct(relativeContactPositions[i], contactNormal);
        Matrix3 inverseInertiaTensor = bodies[i].getInverseInertiaTensorWorld();
        // Work out the direction we'd need to rotate to achieve that
        inverseInertiaTensor.transform(targetAngularDirection);
        angularChanges[i] =
            Vector3D.multiply(targetAngularDirection, angularMoves[i] / angularInertia[i]);
      }
      // Velocity change, linear movement along the contact normal
      linearChanges[i] = new Vector3D(contactNormal);
      linearChanges[i].multiply(linearMoves[i]);
      // Apply linear movement to bodies
      Vector3D position = bodies[i].getPosition();
      position.addScaledVector(contactNormal, linearMoves[i]);
      bodies[i].setPosition(position);
      // Apply the orientation change to bodies
      bodies[i].getOrientation().addScaledVector(angularChanges[i], 1.0);
      // Calculate derived data for any bodies that are asleep, so that the changes are reflected in
      // the object's data. Otherwise the resolution will not change the position of the object and
      // the next collision detection round will have the same penetration.
      if (!bodies[i].isAwake()) {
        bodies[i].calculateDerivedData();
      }
    }
  }

  /**
   * Calculates the impulse needed to resolve this contact, given that the contact has no friction.
   * A pair of inertia tensors - one for each contact object - is specified to save calculation
   * time: the calling method has access to these anyway.
   *
   * <p><b>Note:</b> The provided array has to contain at least one vector. If the contact has only
   * one rigid body, the provided array may also only contain just one vector. If the contact
   * involves two bodies, the array has to contain two vectors. This method has no effect if these
   * conditions are not honored.
   *
   * @param inverseInertiaTensors the inverse inertia tensors of the contact's two rigid bodies
   * @return the calculated impulse or {@code null} if the provided array is invalid
   */
  private Vector3D calculateFrictionlessImpulse(Matrix3[] inverseInertiaTensors) {
    // Validating parameter dimensions and content
    if (bodies[0] != null
        && (inverseInertiaTensors.length < 1 || inverseInertiaTensors[0] == null)) {
      return null;
    } else if (bodies[1] != null
        && (inverseInertiaTensors.length < 2 || inverseInertiaTensors[1] == null)) {
      return null;
    }
    Vector3D impulseContact = new Vector3D();
    // Build vector that shows the change in velocity in world space for a unit impulse in the
    // direction of the contact normal
    Vector3D deltaVelWorld = Vector3D.vectorProduct(relativeContactPositions[0], contactNormal);
    inverseInertiaTensors[0].transform(deltaVelWorld);
    deltaVelWorld.vectorProduct(relativeContactPositions[0]);
    // Work out the change in velocity in contact coordinates
    double deltaVelocity = deltaVelWorld.scalarProduct(contactNormal);
    // Add the linear component of velocity change
    deltaVelocity += bodies[0].getInverseMass();
    // Check if we need to add a second body's data
    if (bodies[1] != null) {
      // Go through same process as for the first body's data
      deltaVelWorld = Vector3D.vectorProduct(relativeContactPositions[1], contactNormal);
      inverseInertiaTensors[1].transform(deltaVelWorld);
      deltaVelWorld.vectorProduct(relativeContactPositions[1]);
      // Add the change in velocity due to rotation
      deltaVelocity += deltaVelWorld.scalarProduct(contactNormal);
      // Add the change in velocity due to linear motion
      deltaVelocity += bodies[1].getInverseMass();
    }
    // Only on x axis, as the contact normal is the x axis of the contact's local coordinates
    impulseContact.setX(desiredDeltaVelocity / deltaVelocity);
    return impulseContact;
  }

  /**
   * Calculates the impulse needed to resolve this contact, given that the contact has a non-zero
   * coefficient of friction. A pair of inertia tensors - one for each contact object - is specified
   * to save calculation time: the calling method has access to these anyway.
   *
   * <p><b>Note:</b> The provided array has to contain at least one vector. If the contact has only
   * one rigid body, the provided array may also only contain just one vector. If the contact
   * involves two bodies, the array has to contain two vectors. This method has no effect if these
   * conditions are not honored.
   *
   * @param inverseInertiaTensors the inverse inertia tensors of the contact's two rigid bodies
   * @return the calculated impulse or {@code null} if the provided array is invalid
   */
  private Vector3D calculateFrictionImpulse(Matrix3[] inverseInertiaTensors) {
    if (bodies[0] != null
        && (inverseInertiaTensors.length < 1 || inverseInertiaTensors[0] == null)) {
      return null;
    } else if (bodies[1] != null
        && (inverseInertiaTensors.length < 2 || inverseInertiaTensors[1] == null)) {
      return null;
    }
    Vector3D impulseContact = new Vector3D();
    double inverseMass = bodies[0].getInverseMass();
    // The equivalent of a cross product in matrices is multiplication by skew symmetric matrix -
    // build the matrix for converting between linear and angular quantities
    Matrix3 impulseToTorque = new Matrix3();
    impulseToTorque.setSkewSymmetric(relativeContactPositions[0]);
    // Build the matrix to convert contact impulse to change in velocity in world coordinates
    Matrix3 deltaVelWorld = new Matrix3(impulseToTorque);
    deltaVelWorld.multiply(inverseInertiaTensors[0]);
    deltaVelWorld.multiply(impulseToTorque);
    deltaVelWorld.multiply(-1);
    // Check if we need to add the second body's data
    if (bodies[1] != null) {
      // Set the cross product matrix
      impulseToTorque.setSkewSymmetric(relativeContactPositions[1]);
      // Calculate the velocity change matrix
      Matrix3 deltaVelWorld2 = new Matrix3(impulseToTorque);
      deltaVelWorld2.multiply(inverseInertiaTensors[1]);
      deltaVelWorld2.multiply(impulseToTorque);
      deltaVelWorld2.multiply(-1);
      // Add to the total delta velocity
      deltaVelWorld.add(deltaVelWorld2);
      // Add to the inverse mass
      inverseMass += bodies[1].getInverseMass();
    }
    // Change of basis to convert into contact coordinates
    Matrix3 deltaVelocity = contactToWorld.transpose();
    deltaVelocity.multiply(deltaVelWorld);
    deltaVelocity.multiply(contactToWorld);
    // Add the linear velocity change
    deltaVelocity.setA(deltaVelocity.getA() + inverseMass);
    deltaVelocity.setE(deltaVelocity.getE() + inverseMass);
    deltaVelocity.setI(deltaVelocity.getI() + inverseMass);
    // Invert to get the impulse needed per unit velocity
    Matrix3 impulseMatrix = deltaVelocity.inverse();
    // Find the target velocities to kill
    Vector3D velKill =
        new Vector3D(desiredDeltaVelocity, -contactVelocity.getY(), -contactVelocity.getZ());
    // Find the impulse to kill target velocities
    impulseMatrix.transform(velKill);
    impulseContact = velKill;
    // Check for exceeding friction
    double planarImpulse =
        Math.sqrt(
            impulseContact.getY() * impulseContact.getY()
                + impulseContact.getZ() * impulseContact.getZ());
    if (planarImpulse > impulseContact.getX() * friction) {
      // Use dynamic friction
      impulseContact.setY(impulseContact.getY() / planarImpulse);
      impulseContact.setZ(impulseContact.getZ() / planarImpulse);
      impulseContact.setX(
          deltaVelocity.getA()
              + deltaVelocity.getB() * friction * impulseContact.getY()
              + deltaVelocity.getC() * friction * impulseContact.getZ());
      impulseContact.setX(desiredDeltaVelocity / impulseContact.getX());
      impulseContact.setY(impulseContact.getY() * friction * impulseContact.getX());
      impulseContact.setZ(impulseContact.getZ() * friction * impulseContact.getX());
    }
    return impulseContact;
  }

  /**
   * Returns the the array in which the two rigid bodies involved in the collision are stored.
   *
   * <p><b>Note:</b> If the contact e.g. describes a collision between a rigid body and a piece of
   * world geometry, one of the entries might be {@code null}.
   *
   * @return an array containing at least one rigid body
   */
  public RigidBody[] getBodies() {
    return bodies;
  }

  /**
   * Returns the first rigid body of possibly two rigid bodies involved in the contact.
   *
   * @return the first of the two rigid bodies
   */
  public RigidBody getBodyA() {
    return bodies[0];
  }

  /**
   * Sets the provided rigid body as the first of the possibly two bodies involved in the contact.
   *
   * @param bodyA a body involved in the contact
   */
  public void setBodyA(RigidBody bodyA) {
    bodies[0] = bodyA;
  }

  /**
   * Returns the second rigid body of possibly two rigid bodies involved in the contact. If there is
   * only one rigid body involved, this method will return {@code null}.
   *
   * @return the second of the two rigid bodies or {@code null}
   */
  public RigidBody getBodyB() {
    return bodies[1];
  }

  /**
   * Sets the provided rigid body as the second of the two bodies involved in the contact.
   *
   * @param bodyB a body involved in the contact
   */
  public void setBodyB(RigidBody bodyB) {
    bodies[1] = bodyB;
  }

  /**
   * Returns the restitution coefficient of the contact.
   *
   * @return the value of {@link #restitution}
   */
  public double getRestitution() {
    return restitution;
  }

  /**
   * Sets the contact's restitution to the provided value.
   *
   * @param restitution the coefficient of restitution
   */
  public void setRestitution(double restitution) {
    this.restitution = restitution;
  }

  /**
   * Returns the point at which the contact occurred in world coordinates.
   *
   * @return {@link #contactPoint}
   */
  public Vector3D getContactPoint() {
    return contactPoint;
  }

  /**
   * Sets the values of the given vector as this contact's contact point.
   *
   * @param contactPoint the point at which the contact occurred in world coordinates
   */
  public void setContactPoint(Vector3D contactPoint) {
    this.contactPoint.setValues(contactPoint);
  }

  /**
   * Sets the values of the contact point vector to the provided values.
   *
   * @param x the contact's x coordinate
   * @param y the contact's y coordinate
   * @param z the contact's z coordinate
   */
  public void setContactPoint(double x, double y, double z) {
    contactPoint.setValues(x, y, z);
  }

  /**
   * Returns the direction of the contact in world coordinates.
   *
   * @return {@link #contactNormal}
   */
  public Vector3D getContactNormal() {
    return contactNormal;
  }

  /**
   * Sets the given vector as this contact's contact normal.
   *
   * @param contactNormal the direction of the contact in world coordinates
   */
  public void setContactNormal(Vector3D contactNormal) {
    this.contactNormal.setValues(contactNormal);
  }

  /**
   * Sets the contact normal values to the provided values.
   *
   * @param x the normal's amount in the x direction
   * @param y the normal's amount in the y direction
   * @param z the normal's amount in the z direction
   */
  public void setContactNormal(double x, double y, double z) {
    contactNormal.setValues(x, y, z);
  }

  /**
   * Returns the penetration depth of the contact.
   *
   * @return the value of {@link #penetration}
   */
  public double getPenetration() {
    return penetration;
  }

  /**
   * Sets the contact's penetration depth to the given value.
   *
   * @param penetration the contact's penetration depth
   */
  public void setPenetration(double penetration) {
    this.penetration = penetration;
  }

  /**
   * Returns the matrix with which coordinates in contact space can be converted to world space.
   *
   * @return {@link #contactToWorld}
   */
  public Matrix3 getContactToWorld() {
    return contactToWorld;
  }

  /**
   * Returns the closing velocity of the point of contact.
   *
   * @return {@link #contactVelocity}
   */
  public Vector3D getContactVelocity() {
    return contactVelocity;
  }

  /**
   * Returns the required change in velocity for this contact to be resolved.
   *
   * @return the value of {@link #desiredDeltaVelocity}
   */
  public double getDesiredDeltaVelocity() {
    return desiredDeltaVelocity;
  }

  /**
   * Returns the world space positions of the contact points relative to the center of their
   * respective rigid body. The vector at index 0 is the position for the first rigid body, the
   * vector at index 1 the position for the second rigid body.
   *
   * <p><b>Note:</b> If the contact e.g. describes a collision between a rigid body and a piece of
   * world geometry, one of the entries might be {@code null}.
   *
   * @return an array containing at least one vector
   */
  public Vector3D[] getRelativeContactPositions() {
    return relativeContactPositions;
  }
}
