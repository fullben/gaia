package de.fullben.processing.gaia.rigidbodies;

import de.fullben.processing.gaia.Configuration;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.contacts.BoundingSphere;
import de.fullben.processing.gaia.rigidbodies.contacts.CollisionData;
import de.fullben.processing.gaia.rigidbodies.contacts.CollisionDetector;
import de.fullben.processing.gaia.rigidbodies.contacts.CollisionPlane;
import de.fullben.processing.gaia.rigidbodies.contacts.ContactResolver;
import de.fullben.processing.gaia.rigidbodies.contacts.PotentialContact;
import de.fullben.processing.gaia.rigidbodies.fgens.ForceGenerator;
import java.util.ArrayList;
import java.util.List;
import processing.core.PShape;

/**
 * The {@code World} represents an independent simulation of physics. It keeps track of a set of
 * rigid bodies, and provides the means to update them all.
 *
 * @author Benedikt Full
 * @see RigidBody
 */
public class World {

  /** The physics entities simulated by this world. */
  private final List<Entity> entities;
  /** The immovable collision geometry of the world. */
  private final List<CollisionPlane> collisionHalfSpaces;
  /** The forces of the world. */
  private final ForceRegistry forceRegistry;
  /** The contact resolver for sets of contacts. */
  private final ContactResolver contactResolver;
  /** The potential contacts found in this world. */
  private final List<PotentialContact> potentialContacts;
  /** The contacts found in this world. */
  private final CollisionData collisionData;
  /** The maximum number of contacts allowed. */
  private final int maxContacts;
  /** The total number of contacts currently occurring within the world. */
  private int currentContacts;
  /** The nanosecond timestamp of the last time the world was updated. */
  private long lastUpdateNanos;

  /**
   * Constructs a new and empty world. Both friction and the coefficient of restitution of all
   * collisions are set to 0.7.
   */
  public World() {
    this(0.7, 0.7, 1000);
  }

  /**
   * Constructs a new and empty world based on the given parameters.
   *
   * @param friction the friction occurring in all collisions
   * @param restitution the coefficient of restitution shared by all objects inhabiting the world
   * @param maxContacts the maximum number of contacts the world may generate per update step
   */
  public World(double friction, double restitution, int maxContacts) {
    entities = new ArrayList<>();
    collisionHalfSpaces = new ArrayList<>();
    forceRegistry = new ForceRegistry();
    contactResolver = new ContactResolver();
    potentialContacts = new ArrayList<>();
    collisionData = new CollisionData(friction, restitution, 0.01);
    this.maxContacts = maxContacts;
    currentContacts = 0;
    lastUpdateNanos = System.nanoTime();
  }

  /**
   * Updates the physics of entities populating this world based on the duration which has elapsed
   * since the last time this method or {@link #update(double)} was called.
   *
   * <p>Note that this method has no effect if the provided duration exceeds the maximum frame
   * duration value specified in the global configuration (available at {@link
   * Configuration#current()}), as processing the physics for longer durations generally only
   * produces undesirable results.
   *
   * @see #update(double)
   */
  public void update() {
    long currentNanoTime = System.nanoTime();
    long timeElapsed = currentNanoTime - lastUpdateNanos;
    lastUpdateNanos = currentNanoTime;
    runPhysics(timeElapsed / 1_000_000_000.0);
  }

  /**
   * Updates the physics of entities populating this world based on the given duration.
   *
   * <p>Note that this method has no effect if the provided duration exceeds the maximum frame
   * duration value specified in the global configuration (available at {@link
   * Configuration#current()}), as processing the physics for longer durations generally only
   * produces undesirable results.
   *
   * @param durationMillis duration in milliseconds which has elapsed since the last time the world
   *     was updated
   * @see #update()
   */
  public void update(double durationMillis) {
    lastUpdateNanos = System.nanoTime();
    runPhysics(durationMillis / 1_000.0);
  }

  /**
   * Processes all the physics for the world.
   *
   * <p>This method has no effect if the duration exceeds the value of {@link
   * Configuration#getMaxFrameDuration()}, as processing the physics for longer durations generally
   * only produces undesirable results.
   *
   * @param duration duration for which to process in seconds, must be greater than zero and may not
   *     exceed the safety limit
   * @see #resetWorld()
   */
  private void runPhysics(double duration) {
    // Validate duration
    if (duration <= 0 || duration > Configuration.current().getMaxFrameDuration()) {
      System.err.println("Attempted to run physics with unsafe frame duration: " + duration + "s");
      return;
    }
    // Apply the force generators
    forceRegistry.updateForces(duration);
    // Integrate the objects
    integrate(duration);
    // Generate contacts
    int usedContacts = generateContacts();
    currentContacts = usedContacts;
    // Resolve contacts
    if (usedContacts > 0) {
      contactResolver.resolveContacts(collisionData.getContacts(), duration);
    }
    resetWorld();
  }

  /**
   * Prepares the world for a simulation frame. This method clears the force and torque accumulators
   * for bodies in the world. After calling this, the bodies can have their forces and torques for a
   * new frame added.
   */
  private void resetWorld() {
    for (Entity e : entities) {
      RigidBody body = e.getRigidBody();
      body.clearAccumulators();
      body.calculateDerivedData();
      e.calculateInternals();
    }
  }

  /**
   * Integrates all the bodies in the world forward in time by the given duration.
   *
   * @param duration duration for which to integrate in seconds
   */
  private void integrate(double duration) {
    for (Entity e : entities) {
      e.getRigidBody().integrate(duration);
      e.calculateInternals();
    }
  }

  /**
   * Generates the potential contacts by checking whether the bounding spheres of the world's
   * entities overlap. Any found contacts are added to {@link #potentialContacts}.
   *
   * @return the total number of generated potential contacts
   */
  private int generatePotentialContacts() {
    int contactCount = 0;
    for (int i = 0; i < entities.size(); i++) {
      BoundingSphere sphereA = entities.get(i).getBoundingSphere();
      for (int j = i + 1; j < entities.size(); j++) {
        BoundingSphere sphereB = entities.get(j).getBoundingSphere();
        if (sphereA.overlaps(sphereB)
            && !duplicatePotentialContact(
                entities.get(i).getRigidBody(), entities.get(j).getRigidBody())) {
          potentialContacts.add(
              new PotentialContact(entities.get(i).getRigidBody(), entities.get(j).getRigidBody()));
          contactCount++;
        }
      }
    }
    return contactCount;
  }

  /**
   * Checks whether a potential contact with the given bodies is already known to the world
   *
   * @param bodyA the first body
   * @param bodyB the second body
   * @return {@code true} if a potential contacts involving the given bodies exists
   */
  private boolean duplicatePotentialContact(RigidBody bodyA, RigidBody bodyB) {
    for (PotentialContact potentialContact : potentialContacts) {
      if (potentialContact.isBetween(bodyA, bodyB)) {
        return true;
      }
    }
    return false;
  }

  /**
   * Generates all contacts involving the immovable world geometry and adds them to {@link
   * #collisionData}. All world geometry objects are stored in {@link #collisionHalfSpaces}.
   *
   * @param limit the maximum number of contacts to generate
   * @return the number of contacts generated
   */
  private int generateHalfSpaceContacts(int limit) {
    for (Entity entity : entities) {
      for (CollisionPlane hs : collisionHalfSpaces) {
        limit -= CollisionDetector.detect(hs, entity.getCollisionGeometry(), collisionData);
        if (limit <= 0) {
          break;
        }
      }
    }
    return limit;
  }

  /**
   * Generates all the contacts occurring in the world. This is done by first checking for any
   * potential contacts, followed by checking for any contacts involving the immovable world
   * geometry. During the final step collisions between entities are checked for and created. All
   * contacts are added to {@link #collisionData}.
   *
   * @return the total number of contacts generated
   */
  private int generateContacts() {
    int potContacts = generatePotentialContacts();
    int limit = maxContacts;
    collisionData.reset();
    limit = generateHalfSpaceContacts(limit);
    for (PotentialContact potentialContact : potentialContacts) {
      RigidBody bodyA = potentialContact.getBodyA();
      RigidBody bodyB = potentialContact.getBodyB();
      Entity entityA = null;
      Entity entityB = null;
      for (Entity entity : entities) {
        if (entity.represents(bodyA)) {
          entityA = entity;
        } else if (entity.represents(bodyB)) {
          entityB = entity;
        }
        // Abandon loop as soon as entities are found
        if (entityA != null && entityB != null) {
          break;
        }
      }
      // Should be able to find all collisions between spheres and boxes
      if (entityA != null && entityB != null) {
        limit -=
            CollisionDetector.detect(
                entityA.getCollisionGeometry(), entityB.getCollisionGeometry(), collisionData);
      }
      if (limit <= 0) {
        break;
      }
    }
    return maxContacts - limit;
  }

  /**
   * Returns a list of all the rigid bodies associated with this world.
   *
   * @return a list of the world's inhabitants
   */
  public List<RigidBody> getRigidBodies() {
    List<RigidBody> bodies = new ArrayList<>();
    for (Entity entity : entities) {
      bodies.add(entity.getRigidBody());
    }
    return bodies;
  }

  /**
   * Adds the provided rigid body to the world. For collision geometry generation, it is assumed
   * that this body is a sphere with the given radius.
   *
   * @param rigidBody a rigid body describing a sphere
   * @param radius the sphere's radius
   * @throws IllegalArgumentException if the given body has been added to the world before
   */
  public void addRigidBody(RigidBody rigidBody, double radius) {
    entities.add(new Entity(requireNotPartOfWorld(rigidBody), radius));
    forceRegistry.register(rigidBody);
  }

  /**
   * Adds the provided rigid body to the world. For collision geometry generation, it is assumed
   * that this body is a cuboid of the given dimensions.
   *
   * @param rigidBody a rigid body describing a cuboid
   * @param xDim the extent of the cuboid on the x axis
   * @param yDim the extent of the cuboid on the y axis
   * @param zDim the extend of the cuboid on the z axis
   * @throws IllegalArgumentException if the given body has been added to the world before
   */
  public void addRigidBody(RigidBody rigidBody, double xDim, double yDim, double zDim) {
    entities.add(new Entity(requireNotPartOfWorld(rigidBody), xDim, yDim, zDim));
    forceRegistry.register(rigidBody);
  }

  /**
   * Adds the given rigid body to the world. The collision geometry of the body is generated based
   * on the given shape data.
   *
   * @param rigidBody a rigid body
   * @param shape the body's rendering geometry
   * @throws IllegalArgumentException if the given body has been added to the world before
   */
  public void addRigidBody(RigidBody rigidBody, PShape shape) {
    entities.add(new Entity(requireNotPartOfWorld(rigidBody), shape));
    forceRegistry.register(rigidBody);
  }

  /**
   * Adds a planar bound to the world. The bound is assumed to be a half-space, which means that
   * objects may only be positioned in front of the bound, but not behind it.
   *
   * @param direction the normal of the bound
   * @param offset the offset of the bound from the origin
   */
  public void addBound(Vector3D direction, double offset) {
    collisionHalfSpaces.add(new CollisionPlane(direction, offset));
  }

  /**
   * Adds the provided force to the world and applies it to all rigid bodies registered with the
   * world at the point of the method call. This force will also affect any bodies added to the
   * world after the force has been added.
   *
   * @param force the force to be added to the world
   */
  public void addForce(ForceGenerator force) {
    forceRegistry.register(force);
  }

  /**
   * Checks whether the given rigid body has a corresponding entity already registered with this
   * world.
   *
   * @param rigidBody the rigid body to check for
   * @return true if an entity with the {@code rigidBody} exists already, false if it does not
   */
  private boolean isPartOfWorld(RigidBody rigidBody) {
    for (Entity entity : entities) {
      if (entity.represents(rigidBody)) {
        return true;
      }
    }
    return false;
  }

  private RigidBody requireNotPartOfWorld(RigidBody rigidBody) {
    if (isPartOfWorld(rigidBody)) {
      throw new IllegalArgumentException("Cannot add a body to world more than once");
    }
    return rigidBody;
  }

  /**
   * Returns the number of contacts currently existing in the world.
   *
   * @return the value of {@link #currentContacts}
   */
  public int getContactCount() {
    return currentContacts;
  }

  /**
   * Returns the number of rigid bodies currently existing in the world.
   *
   * @return the number of rigid bodies
   */
  public int getRigidBodyCount() {
    return entities.size();
  }

  /**
   * Returns the contact resolver responsible for resolving this world's contacts.
   *
   * <p>Access to the resolver is granted in order to allow the user to modify some of its
   * properties.
   *
   * @return {@link #contactResolver}
   * @see ContactResolver#setPositionIterations(int)
   * @see ContactResolver#setVelocityIterations(int)
   * @see ContactResolver#setPositionEpsilon(double)
   * @see ContactResolver#setVelocityEpsilon(double)
   */
  public ContactResolver getContactResolver() {
    return contactResolver;
  }
}
