package de.fullben.processing.gaia.rigidbodies;

import de.fullben.processing.gaia.math.Quaternion;
import de.fullben.processing.gaia.math.Vector3D;
import de.fullben.processing.gaia.rigidbodies.contacts.BoundingSphere;
import de.fullben.processing.gaia.rigidbodies.contacts.CollisionGeometry;
import de.fullben.processing.gaia.rigidbodies.contacts.GeometryGenerator;
import de.fullben.processing.gaia.rigidbodies.contacts.Mesh;
import java.util.List;
import processing.core.PShape;

/**
 * Entities store and provide access to a variety of components of one physics entity. These
 * components are:
 *
 * <ul>
 *   <li>A mesh, containing the vertices and edges of the rendering geometry (derived from a {@code
 *       PShape})
 *   <li>A rigid body, which stores position, orientation, velocity and many other properties
 *   <li>A bounding volume which will be used during coarse collision detection
 *   <li>A number of collision geometry objects which will be utilized to check for collision during
 *       the narrow collision detection phase
 * </ul>
 *
 * Uniting all these components in one class serves the following purposes:
 *
 * <ul>
 *   <li>Connect the physics engine's main object, the rigid body, with the geometrical shapes
 *       utilized in {@code Processing} and the corresponding collision and bounding geometry
 *       without changing the actual {@code RigidBody} class
 *   <li>Provide access to the geometry used in {@code Processing}, which is necessary for automatic
 *       collision and bounding geometry generation
 *   <li>Compensate for the lack of a proper bounding volume hierarchy system
 * </ul>
 *
 * @author Benedikt Full
 */
public class Entity {

  /** The vertices and edge data of the rendering geometry of this entity. */
  private final Mesh mesh;
  /** The location, orientation and movement data of the entity. */
  private final RigidBody rigidBody;
  /** The collision geometry objects of the entity, used during narrow collision detection. */
  private List<CollisionGeometry> collisionGeometry;
  /** The bounding volume of the entity, used during coarse collision detection. */
  private BoundingSphere boundingSphere;

  /**
   * Constructs a new entity representing a rigid body with the given position, orientation and
   * geometry.
   *
   * @param position the position of the body
   * @param orientation the orientation of the body
   * @param shape the entity's rendering geometry
   */
  Entity(Vector3D position, Quaternion orientation, PShape shape) {
    mesh = new Mesh(shape);
    rigidBody = new RigidBody(position, orientation);
    collisionGeometry = GeometryGenerator.generateCollisionGeometry(rigidBody, mesh);
    boundingSphere = GeometryGenerator.generateBoundingSphere(collisionGeometry);
  }

  /**
   * Constructs a new entity representing the given rigid body and shape.
   *
   * @param rigidBody a fully initialized rigid body
   * @param shape the body's rendering geometry
   */
  Entity(RigidBody rigidBody, PShape shape) {
    mesh = new Mesh(shape);
    this.rigidBody = rigidBody;
    collisionGeometry = GeometryGenerator.generateCollisionGeometry(rigidBody, mesh);
    boundingSphere = GeometryGenerator.generateBoundingSphere(collisionGeometry);
  }

  /**
   * Constructs a new entity representing the given rigid body. The body is assumed to be a sphere
   * with the given radius and located at the body's position.
   *
   * <p><b>Note:</b> Entities initialized this way do not have any valid mesh data. Their {@link
   * #mesh} is an empty object.
   *
   * @param rigidBody a fully initialized rigid body
   * @param radius the sphere's radius
   */
  Entity(RigidBody rigidBody, double radius) {
    mesh = new Mesh();
    this.rigidBody = rigidBody;
    collisionGeometry = GeometryGenerator.generateCollisionGeometry(rigidBody, radius);
    boundingSphere = GeometryGenerator.generateBoundingSphere(collisionGeometry);
  }

  /**
   * Constructs a new entity representing the given rigid body. The body is assumed to be a box with
   * the given dimensions and located at the body's position.
   *
   * <p><b>Note:</b> Entities initialized this way do not have any valid mesh data. Their {@link
   * #mesh} is an empty object.
   *
   * @param rigidBody a fully initialized rigid body
   * @param xDim the extent of the cuboid along the x axis
   * @param yDim the extent of the cuboid along the y axis
   * @param zDim the extent of the cuboid along the z axis
   */
  Entity(RigidBody rigidBody, double xDim, double yDim, double zDim) {
    mesh = new Mesh();
    this.rigidBody = rigidBody;
    collisionGeometry = GeometryGenerator.generateCollisionGeometry(rigidBody, xDim, yDim, zDim);
    boundingSphere = GeometryGenerator.generateBoundingSphere(collisionGeometry);
  }

  /** Updates the internal data of the collision geometry of this entity. */
  protected void calculateInternals() {
    for (CollisionGeometry cg : collisionGeometry) {
      cg.calculateInternals();
    }
  }

  /**
   * Sets the provided geometry as the entity's new rendering geometry and re-generates the bounding
   * and collision geometry.
   *
   * @param shape the new rendering geometry of the entity
   */
  public void updateMesh(PShape shape) {
    mesh.updateGeometry(shape);
    collisionGeometry = GeometryGenerator.generateCollisionGeometry(rigidBody, mesh);
    boundingSphere = GeometryGenerator.generateBoundingSphere(collisionGeometry);
  }

  /**
   * Returns the entity's rigid body's position.
   *
   * @return the entity's position
   */
  public Vector3D getPosition() {
    return rigidBody.getPosition();
  }

  /**
   * Sets the entity's position by component.
   *
   * @param x the x coordinate of the new position
   * @param y the y coordinate of the new position
   * @param z the z coordinate of the new position
   */
  public void setPosition(double x, double y, double z) {
    rigidBody.setPosition(x, y, z);
  }

  /**
   * Returns the entity's rigid body's orientation.
   *
   * @return the entity's orientation
   */
  public Quaternion getOrientation() {
    return rigidBody.getOrientation();
  }

  /**
   * Sets the entity's orientation by component. The orientation is stored as a {@link Quaternion}.
   *
   * @param r the real component
   * @param i the first complex component
   * @param j the second complex component
   * @param k the third complex component
   */
  public void setOrientation(double r, double i, double j, double k) {
    rigidBody.setOrientation(r, i, j, k);
  }

  /**
   * Returns the mesh object associated with this entity. The mesh usually stores the vertices and
   * edges of the rendering geometry represented by this entity. If the entity represents a simple
   * shape (cuboid or sphere), the returned mesh might not contain any geometrical data at all.
   *
   * @return {@link #mesh}
   */
  public Mesh getMesh() {
    return mesh;
  }

  /**
   * Returns the rigid body associated with this entity. The rigid body contains the physics
   * entity's most important data, like position, orientation, velocity etc.
   *
   * @return {@link #rigidBody}
   */
  public RigidBody getRigidBody() {
    return rigidBody;
  }

  /**
   * This method can be used to verify whether this entity represents a given rigid body.
   *
   * <p>An entity represents a given rigid body {@code b} if its {@link #getRigidBody()} method
   * returns a body equal to {@code b}.
   *
   * @param rigidBody a rigid body, may be {@code null}
   * @return {@code true} if this entity represents the given rigid body, {@code false} in any other
   *     case
   */
  public boolean represents(RigidBody rigidBody) {
    return this.rigidBody.equals(rigidBody);
  }

  /**
   * Returns the collision geometry of this entity. In most cases, this will be just one collision
   * geometry object.
   *
   * @return {@link #collisionGeometry}
   */
  public List<CollisionGeometry> getCollisionGeometry() {
    return collisionGeometry;
  }

  /**
   * Returns the bounding sphere of this entity. This sphere encompasses all collision geometry
   * objects associated with this entity.
   *
   * @return {@link #boundingSphere}
   */
  public BoundingSphere getBoundingSphere() {
    return boundingSphere;
  }
}
