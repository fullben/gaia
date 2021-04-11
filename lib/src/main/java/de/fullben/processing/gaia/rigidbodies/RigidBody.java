package de.fullben.processing.gaia.rigidbodies;

import de.fullben.processing.gaia.Configuration;
import de.fullben.processing.gaia.math.Matrix3;
import de.fullben.processing.gaia.math.Matrix4;
import de.fullben.processing.gaia.math.Quaternion;
import de.fullben.processing.gaia.math.Vector3D;

/**
 * A rigid body is the basic simulation object in the physics engine.
 *
 * @author Benedikt Full
 */
public class RigidBody {

  /** The linear position of the rigid body in world space. */
  private final Vector3D position;
  /** The angular orientation of the rigid body in world space. */
  private final Quaternion orientation;
  /** The linear velocity of the rigid body in world space. */
  private final Vector3D velocity;
  /**
   * The acceleration of the rigid body. This value can be used to set acceleration due to gravity
   * (its primary use), or any other constant acceleration.
   */
  private final Vector3D acceleration;
  /** The angular velocity, or rotation, of the rigid body in space. */
  private final Vector3D angularVelocity;
  /** The accumulated force to be applied at the next integration step. */
  private final Vector3D forceAccumulator;
  /** The accumulated torque to be applied at the next integration step. */
  private final Vector3D torqueAccumulator;
  /** The linear acceleration of the rigid body, for the previous frame. */
  private final Vector3D lastFrameAcceleration;
  /**
   * The transform matrix for converting body space into world space and vice versa. This can be
   * achieved by calling {@link #getPointInBodySpace(Vector3D)} or {@link
   * #getPointInWorldSpace(Vector3D)}.
   */
  private final Matrix4 transformMatrix;
  /**
   * The inverse mass of the rigid body. Storing the inverse value instead of the actual is more
   * useful as the integration of the inverse mass is much simpler.
   */
  private double inverseMass;
  /**
   * The inverse of the rigid body's inertia tensor. The inertia tensor provided must not be
   * degenerate (that would mean the body had zero inertia for spinning along one axis). As long as
   * the tensor is finite, it will be invertible. The inverse tensor is used for similar reasons to
   * the use of inverse mass (see {@link #inverseMass}).
   *
   * <p>The inertia tensor, unlike the other variables that define a rigid body, is given in body
   * space.
   */
  private final Matrix3 inverseInertiaTensor;
  /**
   * The inverse inertia tensor of the body in world space. The {@link #inverseInertiaTensor} is
   * specified in the body's local space.
   */
  private final Matrix3 inverseInertiaTensorWorld;
  /**
   * The amount of damping applied to linear motion. Damping is required to remove energy added
   * through numerical instability in the integrator.
   */
  private double linearDamping;
  /**
   * The amount of damping applied to angular motion. Damping is required to remove energy added
   * through numerical instability in the integrator.
   */
  private double angularDamping;
  /**
   * The amount of motion of the body. This is a recency weighted mean that can be used to put a
   * body to sleep.
   */
  private double motion;
  /**
   * Some bodies may never be allowed to fall asleep. User controlled bodies, for example, should be
   * always awake.
   */
  private boolean canSleep;
  /**
   * A rigid body can be put to sleep to avoid it being updated by the integration functions or
   * affected by collisions with the world.
   */
  private boolean isAwake;

  /**
   * Creates a new rigid body. The body's properties will default to the following values:
   *
   * <ul>
   *   <li>Position: Origin
   *   <li>Orientation: Default
   *   <li>Velocity: None
   *   <li>Acceleration: None
   *   <li>Mass: 1 kilogram (Use {@link #setMass(double)} or {@link #setInverseMass(double)} to
   *       adjust the object's mass)
   *   <li>Inertia behavior: 1 meter cube (Use one of the {@code setInertiaTensor...} methods to
   *       change the behavior)
   * </ul>
   */
  public RigidBody() {
    this(new Vector3D(), new Quaternion());
  }

  /**
   * Creates a new rigid body with the given parameters. Furthermore, various properties will
   * default to the following values:
   *
   * <ul>
   *   <li>Position: As provided
   *   <li>Orientation: As provided
   *   <li>Velocity: None
   *   <li>Acceleration: None
   *   <li>Mass: 1 kilogram (Use {@link #setMass(double)} or {@link #setInverseMass(double)} to
   *       adjust the object's mass)
   *   <li>Inertia behavior: 1 meter cube (Use one of the {@code setInertiaTensor...} methods to
   *       change the behavior)
   * </ul>
   *
   * @param position the location of the body's center of mass
   * @param orientation the orientation of the body
   */
  public RigidBody(Vector3D position, Quaternion orientation) {
    this.position = position;
    this.orientation = orientation;
    velocity = new Vector3D();
    acceleration = new Vector3D();
    angularVelocity = new Vector3D();
    forceAccumulator = new Vector3D();
    torqueAccumulator = new Vector3D();
    lastFrameAcceleration = new Vector3D();
    transformMatrix = new Matrix4(); // calculateDerivedData()
    inverseInertiaTensor = new Matrix3(); // setInertiaTensor...
    inverseInertiaTensorWorld = new Matrix3(); // calculateDerivedData()
    inverseMass = 1;
    linearDamping = 0.9;
    angularDamping = 0.9;
    motion = 0;
    canSleep = true;
    isAwake = true;
    calculateDerivedData();
    setInertiaTensorCuboid(1, 1, 1);
  }

  /**
   * Creates a transform matrix from the provided position and orientation and stores it in the
   * given matrix.
   *
   * @param transformMatrix the target matrix object
   * @param pos the position
   * @param o the orientation
   */
  private static void calculateTransformMatrix(
      Matrix4 transformMatrix, Vector3D pos, Quaternion o) {
    transformMatrix.setA(1 - 2 * o.getJ() * o.getJ() - 2 * o.getK() * o.getK());
    transformMatrix.setB(2 * o.getI() * o.getJ() - 2 * o.getR() * o.getK());
    transformMatrix.setC(2 * o.getI() * o.getK() + 2 * o.getR() * o.getJ());
    transformMatrix.setD(pos.getX());

    transformMatrix.setE(2 * o.getI() * o.getJ() + 2 * o.getR() * o.getK());
    transformMatrix.setF(1 - 2 * o.getI() * o.getI() - 2 * o.getK() * o.getK());
    transformMatrix.setG(2 * o.getJ() * o.getK() - 2 * o.getR() * o.getI());
    transformMatrix.setH(pos.getY());

    transformMatrix.setI(2 * o.getI() * o.getK() - 2 * o.getR() * o.getJ());
    transformMatrix.setJ(2 * o.getJ() * o.getK() + 2 * o.getR() * o.getI());
    transformMatrix.setK(1 - 2 * o.getI() * o.getI() - 2 * o.getJ() * o.getJ());
    transformMatrix.setL(pos.getZ());
  }

  /**
   * Internal method capable of transforming an inertia tensor by a quaternion.
   *
   * <p><b>Note:</b> Although the original description of this method claims that it is capable of
   * transforming an inertia tensor by a quaternion, it never actually utilizes the provided - or
   * any other - quaternion. This implementation might thus be wrong and not fulfill its supposed
   * purpose.
   *
   * @param iitWorld inverse inertia tensor in world space
   * @param q not utilized quaternion parameter
   * @param iitBody inverse inertia tensor in body space
   * @param rotMatrix transform matrix
   */
  private static void transformInertiaTensor(
      Matrix3 iitWorld, Quaternion q, Matrix3 iitBody, Matrix4 rotMatrix) {
    double t4 =
        rotMatrix.getA() * iitBody.getA()
            + rotMatrix.getB() * iitBody.getD()
            + rotMatrix.getC() * iitBody.getG();
    double t9 =
        rotMatrix.getA() * iitBody.getB()
            + rotMatrix.getB() * iitBody.getE()
            + rotMatrix.getC() * iitBody.getH();
    double t14 =
        rotMatrix.getA() * iitBody.getC()
            + rotMatrix.getB() * iitBody.getF()
            + rotMatrix.getC() * iitBody.getI();

    double t28 =
        rotMatrix.getE() * iitBody.getA()
            + rotMatrix.getF() * iitBody.getD()
            + rotMatrix.getG() * iitBody.getG();
    double t33 =
        rotMatrix.getE() * iitBody.getB()
            + rotMatrix.getF() * iitBody.getE()
            + rotMatrix.getG() * iitBody.getH();
    double t38 =
        rotMatrix.getE() * iitBody.getC()
            + rotMatrix.getF() * iitBody.getF()
            + rotMatrix.getG() * iitBody.getI();

    double t52 =
        rotMatrix.getI() * iitBody.getA()
            + rotMatrix.getJ() * iitBody.getD()
            + rotMatrix.getK() * iitBody.getG();
    double t57 =
        rotMatrix.getI() * iitBody.getB()
            + rotMatrix.getJ() * iitBody.getE()
            + rotMatrix.getK() * iitBody.getH();
    double t62 =
        rotMatrix.getI() * iitBody.getC()
            + rotMatrix.getJ() * iitBody.getF()
            + rotMatrix.getK() * iitBody.getH();

    iitWorld.setA(t4 * rotMatrix.getA() + t9 * rotMatrix.getB() + t14 * rotMatrix.getC());
    iitWorld.setB(t4 * rotMatrix.getE() + t9 * rotMatrix.getF() + t14 * rotMatrix.getG());
    iitWorld.setC(t4 * rotMatrix.getI() + t9 * rotMatrix.getJ() + t14 * rotMatrix.getK());
    iitWorld.setD(t28 * rotMatrix.getA() + t33 * rotMatrix.getB() + t38 * rotMatrix.getC());
    iitWorld.setE(t28 * rotMatrix.getE() + t33 * rotMatrix.getF() + t38 * rotMatrix.getG());
    iitWorld.setF(t28 * rotMatrix.getI() + t33 * rotMatrix.getJ() + t38 * rotMatrix.getK());
    iitWorld.setG(t52 * rotMatrix.getA() + t57 * rotMatrix.getB() + t62 * rotMatrix.getC());
    iitWorld.setH(t52 * rotMatrix.getE() + t57 * rotMatrix.getF() + t62 * rotMatrix.getG());
    iitWorld.setI(t52 * rotMatrix.getI() + t57 * rotMatrix.getJ() + t62 * rotMatrix.getK());

    /* Possible fix for the faulty inverse inertia tensor transformation
    Quaternion tmp = new Quaternion(0, iitWorld.getA(), iitWorld.getD(), iitWorld.getG());
    tmp.multiply(q);
    iitWorld.setA(tmp.getI());
    iitWorld.setD(tmp.getJ());
    iitWorld.setG(tmp.getK());

    tmp.setR(0);
    tmp.setI(iitWorld.getB());
    tmp.setJ(iitWorld.getE());
    tmp.setK(iitWorld.getH());
    tmp.multiply(q);
    iitWorld.setB(tmp.getI());
    iitWorld.setE(tmp.getJ());
    iitWorld.setH(tmp.getK());

    tmp.setR(0);
    tmp.setI(iitWorld.getC());
    tmp.setJ(iitWorld.getF());
    tmp.setK(iitWorld.getI());
    tmp.multiply(q);
    iitWorld.setC(tmp.getI());
    iitWorld.setF(tmp.getJ());
    iitWorld.setI(tmp.getK());*/
  }

  /**
   * Integrates the rigid body forward in time by the provided amount. This method uses a
   * Newton-Euler integration method, which is a linear approximation to the correct integral. For
   * this reason it may be inaccurate in certain cases.
   *
   * @param duration time for which to integrate in seconds
   */
  public void integrate(double duration) {
    // Early out if body is asleep
    if (!isAwake) {
      return;
    }
    // Calculate linear acceleration from force inputs
    lastFrameAcceleration.setValues(acceleration);
    lastFrameAcceleration.addScaledVector(forceAccumulator, inverseMass);
    // Calculate angular acceleration from torque inputs
    Vector3D angularAcceleration = Matrix3.transform(inverseInertiaTensorWorld, torqueAccumulator);
    // Update linear velocity from both acceleration and impulse
    velocity.addScaledVector(lastFrameAcceleration, duration);
    // Update angular velocity from both acceleration and impulse
    angularVelocity.addScaledVector(angularAcceleration, duration);
    // Impose drag
    velocity.multiply(Math.pow(linearDamping, duration));
    angularVelocity.multiply(Math.pow(angularDamping, duration));
    // Update linear position
    position.addScaledVector(velocity, duration);
    // Update angular position
    orientation.addScaledVector(angularVelocity, duration);
    // Normalize orientation and update matrices with new position and orientation
    calculateDerivedData();
    // Clear accumulated forces and torques
    clearAccumulators();
    // Update the kinetic energy store, and possibly put the body to sleep
    if (canSleep) {
      double currentMotion =
          velocity.scalarProduct(velocity) + angularVelocity.scalarProduct(angularVelocity);
      double bias = Math.pow(0.5, duration);
      motion = bias * motion + (1 - bias) * currentMotion;
      if (motion < Configuration.getConfig().getSleepEpsilon()) {
        setIsAwake(false);
      } else if (motion > 10 * Configuration.getConfig().getSleepEpsilon()) {
        motion = 10 * Configuration.getConfig().getSleepEpsilon();
      }
    }
  }

  /**
   * Clears the forces and torques in the accumulators of this rigid body. This will be called
   * automatically after each integration step.
   */
  public void clearAccumulators() {
    forceAccumulator.clear();
    torqueAccumulator.clear();
  }

  /**
   * Calculates internal data from state data. This should be called after the body's state is
   * altered directly (it is called automatically during integration). Thus, if the body's state is
   * changed and then integrated before querying any data (such as {@link #transformMatrix}, calling
   * this method is not necessary.
   */
  public void calculateDerivedData() {
    orientation.normalize();
    // Calculate transform matrix for this body
    calculateTransformMatrix(transformMatrix, position, orientation);
    // Calculate inertia tensor in world space
    transformInertiaTensor(
        inverseInertiaTensorWorld, orientation, inverseInertiaTensor, transformMatrix);
  }

  /**
   * Converts the provided point from world space into the body's local space.
   *
   * @param point point in world space
   * @return new vector containing the {@code point} in the local space
   */
  public Vector3D getPointInBodySpace(Vector3D point) {
    return transformMatrix.transformInverse(point);
  }

  /**
   * Converts the provided point from body space into the world space.
   *
   * @param point point in body space
   * @return new vector containing the {@code point} in world space
   */
  public Vector3D getPointInWorldSpace(Vector3D point) {
    return Matrix4.transform(transformMatrix, point);
  }

  /**
   * Adds the given force to the center of mass of the rigid body. The force is expressed in world
   * coordinates.
   *
   * @param force the force to be applied
   */
  public void addForce(Vector3D force) {
    forceAccumulator.add(force);
    isAwake = true;
  }

  /**
   * Adds the provided force to the given point on the rigid body. Both the force and the
   * application point are given in world space. Because the force is not applied at the center of
   * mass, it may be split into both a force and a torque.
   *
   * @param force force in world space
   * @param point point on the rigid body in world space
   */
  public void addForceAtPoint(Vector3D force, Vector3D point) {
    // Convert to coordinates relative to center of mass
    Vector3D pt = new Vector3D(point);
    pt.subtract(position);

    forceAccumulator.add(force);
    pt.vectorProduct(force);
    torqueAccumulator.add(pt);

    isAwake = true;
  }

  /**
   * Adds the provided force to the given point on the rigid body. The direction of the force is
   * given in world coordinates, but the application point is given in body space. This is useful
   * for spring forces, or other forces fixed to the body.
   *
   * @param force force in world space
   * @param point point on the rigid body in body space
   */
  public void addForceAtBodyPoint(Vector3D force, Vector3D point) {
    // Convert to coordinates relative to center of mass
    Vector3D pt = getPointInWorldSpace(point);
    addForceAtPoint(force, pt);

    isAwake = true;
  }

  /**
   * Adds the provided velocity to the current velocity of the rigid body.
   *
   * @param velocity the additional velocity
   */
  public void addVelocity(Vector3D velocity) {
    this.velocity.add(velocity);
  }

  /**
   * Adds the provided velocity to the current angular velocity of the rigid body.
   *
   * @param angularVelocity the additional velocity
   */
  public void addAngularVelocity(Vector3D angularVelocity) {
    this.angularVelocity.add(angularVelocity);
  }

  /**
   * Returns the inverse mass of this rigid body.
   *
   * @return value of {@link #inverseMass}
   */
  public double getInverseMass() {
    return inverseMass;
  }

  /**
   * Sets the value of the {@link #inverseMass} field to the provided value.
   *
   * @param inverseMass the new inverse mass of the body
   */
  public void setInverseMass(double inverseMass) {
    this.inverseMass = inverseMass;
  }

  /**
   * Returns the mass of the body. If the mass is infinite, this method will return {@value
   * Double#MAX_VALUE}.
   *
   * @return the {@code RigidBody}'s mass
   */
  public double getMass() {
    if (inverseMass == 0.0) {
      return Double.MAX_VALUE;
    } else {
      return 1 / inverseMass;
    }
  }

  /**
   * Sets the mass of this body to the provided value.
   *
   * @param mass a non-zero value
   * @throws IllegalArgumentException if the given mass value is zero
   */
  public void setMass(double mass) {
    if (mass == 0.0) {
      throw new IllegalArgumentException("Zero mass is not allowed");
    }
    inverseMass = 1 / mass;
  }

  /**
   * Returns true if the mass of the body is finite.
   *
   * <p>A {@code RigidBody}'s mass is finite as long as the value of {@link #inverseMass} is larger
   * or equal to zero.
   *
   * @return true if mass is finite
   */
  public boolean hasFiniteMass() {
    return inverseMass >= 0.0;
  }

  /**
   * Returns the inverse inertia tensor of the body in world space.
   *
   * @return {@link #inverseInertiaTensorWorld}
   */
  public Matrix3 getInverseInertiaTensorWorld() {
    return inverseInertiaTensorWorld;
  }

  /**
   * Sets the inertia tensor for this rigid body.
   *
   * @param inertiaTensor the inertia tensor for the rigid body, must be a full rank matrix and must
   *     be invertible
   */
  public void setInertiaTensor(Matrix3 inertiaTensor) {
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor of the rigid body to a matrix representing the tensor of a rectangular
   * six-sided object of constant density and of the provided dimensions.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param dx the extent of the cuboid along the x axis
   * @param dy the extent of the cuboid along the y axis
   * @param dz the extent of the cuboid along the z axis
   */
  public void setInertiaTensorCuboid(double dx, double dy, double dz) {
    Matrix3 inertiaTensor = new Matrix3();
    double massFactor = (1.0 / 12.0) * getMass();
    inertiaTensor.setA(massFactor * (dy * dy + dz * dz));
    inertiaTensor.setE(massFactor * (dx * dx + dz * dz));
    inertiaTensor.setI(massFactor * (dx * dx + dy * dy));
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a sphere with the given
   * radius and with constant density.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param radius the radius of the solid sphere
   * @see #setInertiaTensorSphereHollow(double)
   */
  public void setInertiaTensorSphereSolid(double radius) {
    Matrix3 inertiaTensor = new Matrix3();
    double tensorValue = 0.4 * getMass() * (radius * radius);
    inertiaTensor.setA(tensorValue);
    inertiaTensor.setE(tensorValue);
    inertiaTensor.setI(tensorValue);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a hollow sphere with the
   * given radius.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param radius the radius of the hollow sphere
   * @see #setInertiaTensorSphereSolid(double)
   */
  public void setInertiaTensorSphereHollow(double radius) {
    Matrix3 inertiaTensor = new Matrix3();
    double tensorValue = (2.0 / 3.0) * getMass() * (radius * radius);
    inertiaTensor.setA(tensorValue);
    inertiaTensor.setE(tensorValue);
    inertiaTensor.setI(tensorValue);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of an ellipsoid with the given
   * radii. An ellipsoid is the spherical equivalent of a general cuboid - it can have different
   * radii in each of its three principal axes.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param radiusX the radius along the x axis
   * @param radiusY the radius along the y axis
   * @param radiusZ the radius along the z axis
   */
  public void setInertiaTensorEllipsoid(double radiusX, double radiusY, double radiusZ) {
    Matrix3 inertiaTensor = new Matrix3();
    double massFactor = 0.2 * getMass();
    inertiaTensor.setA(massFactor * (radiusY * radiusY + radiusZ * radiusZ));
    inertiaTensor.setE(massFactor * (radiusX * radiusX + radiusZ * radiusZ));
    inertiaTensor.setI(massFactor * (radiusX * radiusX + radiusY * radiusY));
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a cylinder with the given
   * dimensions. The method assumes that the cylinder has a constant density.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param height the cylinder's height
   * @param radius the cylinder's radius
   * @see #setInertiaTensorCylinderHollow(double, double, double)
   */
  public void setInertiaTensorCylinderSolid(double height, double radius) {
    Matrix3 inertiaTensor = new Matrix3();
    double massHeight = getMass() * (height * height);
    double massRadius = getMass() * (radius * radius);
    double ai = (1.0 / 12.0) * massHeight + 0.25 * massRadius;
    inertiaTensor.setA(ai);
    inertiaTensor.setE(0.5 * massRadius);
    inertiaTensor.setI(ai);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a cylinder with the given
   * dimensions. The cylinder is not solid but instead represents a cylindrical tube.
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param height the cylinder's height
   * @param outerRadius the cylinder's outer radius
   * @param innerRadius the cylinder's inner radius
   * @see #setInertiaTensorCylinderSolid(double, double)
   */
  public void setInertiaTensorCylinderHollow(
      double height, double outerRadius, double innerRadius) {
    Matrix3 inertiaTensor = new Matrix3();
    double mass = getMass();
    double massHeight = (1.0 / 12.0) * mass * (height * height);
    double radiusFactor = outerRadius * outerRadius + innerRadius * innerRadius;
    inertiaTensor.setA(massHeight + 0.25 * mass * radiusFactor);
    inertiaTensor.setE(0.5 * mass * radiusFactor);
    inertiaTensor.setI(massHeight + 0.25 * mass * radiusFactor);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a cone with the given
   * dimensions.
   *
   * <p>The method makes the following assumptions about the cone:
   *
   * <ul>
   *   <li>Its center of mass is located one-quarter of the way from the center of its base to its
   *       tip
   *   <li>The center of the base of the cone is located at 0, -0.25, 0
   *   <li>The cone is oriented so that its tip points along the positive y direction
   * </ul>
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param height the cone's height
   * @param radius the radius at the base of the cone
   */
  public void setInertiaTensorCone(double height, double radius) {
    Matrix3 inertiaTensor = new Matrix3();
    double massHeight = getMass() * (height * height);
    double massRadius = getMass() * (radius * radius);
    inertiaTensor.setA((3 / 80f) * massHeight + (3 / 20f) * massRadius);
    inertiaTensor.setE((3 / 10f) * massRadius);
    inertiaTensor.setI(0.6 * massHeight + (3 / 20f) * massRadius);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Sets the inertia tensor to a matrix corresponding to the tensor of a hemisphere with the given
   * radius.
   *
   * <p>The method makes the following assumptions about the hemisphere:
   *
   * <ul>
   *   <li>Its center of mass is three-eights of the way from the center of its base to its radius
   *       perpendicular to that base
   *   <li>It is placed with its flat surface on an x-z plane
   * </ul>
   *
   * <p><b>Note:</b> Set the body's mass before calling this method.
   *
   * @param radius the radius of the base of the hemisphere
   */
  public void setInertiaTensorHemisphere(double radius) {
    Matrix3 inertiaTensor = new Matrix3();
    double massRadius = getMass() * (radius * radius);
    inertiaTensor.setA((83 / 320f) * massRadius);
    inertiaTensor.setE(0.4 * massRadius);
    inertiaTensor.setI((83 / 320f) * massRadius);
    inverseInertiaTensor.setInverse(inertiaTensor);
  }

  /**
   * Returns a copy of the position of the body in world space.
   *
   * @return a copy of {@link #position}
   */
  public Vector3D getPosition() {
    return new Vector3D(position);
  }

  /**
   * Sets the position of the rigid body by component.
   *
   * @param x the x coordinate of the new position
   * @param y the y coordinate of the new position
   * @param z the z coordinate of the new position
   */
  public void setPosition(double x, double y, double z) {
    position.setValues(x, y, z);
  }

  public void setPosition(Vector3D position) {
    this.position.setValues(position);
  }

  /**
   * Gets the orientation of the rigid body.
   *
   * @return the orientation of the body
   */
  public Quaternion getOrientation() {
    return orientation;
  }

  /**
   * Sets the orientation of the rigid body by component.
   *
   * @param r the real component of the orientation quaternion
   * @param i the first complex component of the orientation quaternion
   * @param j the second complex component of the orientation quaternion
   * @param k the third complex component of the orientation quaternion
   * @see Quaternion
   */
  public void setOrientation(double r, double i, double j, double k) {
    orientation.setR(r);
    orientation.setI(i);
    orientation.setJ(j);
    orientation.setK(k);
  }

  /**
   * Returns a copy of the vector representing the current velocity of the body.
   *
   * @return a copy of {@link #velocity}
   */
  public Vector3D getVelocity() {
    return new Vector3D(velocity);
  }

  /**
   * Sets the velocity of the body by component. The velocity is given in world space.
   *
   * @param x the x coordinate of the new velocity
   * @param y the y coordinate of the new velocity
   * @param z the z coordinate of the new velocity
   */
  public void setVelocity(double x, double y, double z) {
    velocity.setValues(x, y, z);
  }

  /**
   * Returns a copy of the vector representing the current acceleration of the rigid body.
   *
   * @return the rigid body's acceleration in world space
   */
  public Vector3D getAcceleration() {
    return new Vector3D(acceleration);
  }

  /**
   * Sets the constant acceleration of the body to the values of the given vector.
   *
   * @param acceleration the new acceleration of the rigid body
   */
  public void setAcceleration(Vector3D acceleration) {
    this.acceleration.setValues(acceleration);
  }

  /**
   * Sets the constant acceleration of this rigid body by component.
   *
   * @param x the x coordinate of the new acceleration
   * @param y the y coordinate of the new acceleration
   * @param z the z coordinate of the new acceleration
   */
  public void setAcceleration(double x, double y, double z) {
    acceleration.setValues(x, y, z);
  }

  /**
   * Gets the rotation of the rigid body. The rotation is given in world space.
   *
   * @return the rotation of the rigid body
   */
  public Vector3D getAngularVelocity() {
    return angularVelocity;
  }

  /**
   * Sets the angular velocity (rotation) of the body by component. It is given in world space.
   *
   * @param x the x coordinate of the new angular velocity
   * @param y the y coordinate of the new angular velocity
   * @param z the z coordinate of the new angular velocity
   */
  public void setAngularVelocity(double x, double y, double z) {
    angularVelocity.setValues(x, y, z);
  }

  /**
   * Returns the acceleration experienced by the rigid body during the previous frame.
   *
   * @return {@link #lastFrameAcceleration}
   */
  public Vector3D getLastFrameAcceleration() {
    return lastFrameAcceleration;
  }

  /**
   * Returns the transformation matrix which can be used for converting body space into world space
   * and vice versa.
   *
   * @return {@link #transformMatrix}
   */
  public Matrix4 getTransformMatrix() {
    return transformMatrix;
  }

  /**
   * Sets both linear and angular damping in one one method call.
   *
   * @param linearDamping the speed that velocity is shed from the rigid body
   * @param angularDamping the speed that rotation is shed from the rigid body
   * @see #setLinearDamping(double)
   * @see #setAngularDamping(double)
   */
  public void setDamping(double linearDamping, double angularDamping) {
    this.linearDamping = linearDamping;
    this.angularDamping = angularDamping;
  }

  /**
   * Gets the current linear damping value of this body.
   *
   * @return the current linear damping value
   */
  public double getLinearDamping() {
    return linearDamping;
  }

  /**
   * Sets the linear damping for the body.
   *
   * @param linearDamping the speed that velocity is shed from the rigid body
   * @see #setAngularDamping(double)
   * @see #setDamping(double, double)
   */
  public void setLinearDamping(double linearDamping) {
    this.linearDamping = linearDamping;
  }

  /**
   * Gets the current angular damping value of this body.
   *
   * @return the current angular damping value
   */
  public double getAngularDamping() {
    return angularDamping;
  }

  /**
   * Sets the angular damping for the body.
   *
   * @param angularDamping the speed that rotation is shed from the rigid body
   * @see #setLinearDamping(double)
   * @see #setDamping(double, double)
   */
  public void setAngularDamping(double angularDamping) {
    this.angularDamping = angularDamping;
  }

  /**
   * Sets whether the body is ever allowed to go to sleep. Bodies under the player's control, or for
   * which the set of transient forces applied each frame are not predictable should be kept awake.
   *
   * @param canSleep whether the body can be put to sleep
   */
  public void setCanSleep(boolean canSleep) {
    this.canSleep = canSleep;
  }

  /**
   * Returns true if the body is awake and responding to integration.
   *
   * @return the awake state of this body
   */
  public boolean isAwake() {
    return isAwake;
  }

  /**
   * Sets the awake state of the rigid body. If the body is set to be not awake, then its velocities
   * are also cancelled, since a moving body that is not awake can cause problems in the simulation.
   *
   * @param isAwake the new awake state of the body
   */
  public void setIsAwake(boolean isAwake) {
    if (isAwake) {
      this.isAwake = true;
      // Add a bit of motion to avoid it falling asleep immediately
      motion = Configuration.getConfig().getSleepEpsilon() * 2.0f;
    } else {
      this.isAwake = false;
      velocity.clear();
      angularVelocity.clear();
    }
  }
}
