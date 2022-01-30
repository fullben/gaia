package de.fullben.processing.gaia.rigidbodies;

import de.fullben.processing.gaia.math.Quaternion;
import de.fullben.processing.gaia.math.Vector3D;

/**
 * This class hosts all major components that make up the step builder for constructing {@link
 * RigidBody} instances.
 *
 * @author Benedikt Full
 */
public class RigidBodyBuilder {

  /**
   * The actual {@link RigidBody} step builder implementation.
   *
   * @author Benedikt Full
   */
  public static class Builder
      implements PositionStep,
          MassStep,
          OrientationStep,
          VelocityStep,
          AccelerationStep,
          ShapeStep,
          FillStep,
          FinalStep {
    private InertiaTensor inertiaTensor;
    private boolean solid;
    private double inverseMass;
    private Vector3D position;
    private Quaternion orientation;
    private Vector3D velocity;
    private Vector3D acceleration;

    @Override
    public MassStep cuboid(double x, double y, double z) {
      inertiaTensor = new CuboidInertiaTensor(x, z, y);
      return this;
    }

    @Override
    public FillStep sphere(double radius) {
      inertiaTensor = new SphereInertiaTensor(radius);
      return this;
    }

    @Override
    public FillStep cylinder(double height, double radius) {
      inertiaTensor = new CylinderInertiaTensor(height, radius);
      return this;
    }

    @Override
    public MassStep hemisphere(double radius) {
      inertiaTensor = new HemisphereInertiaTensor(radius);
      return this;
    }

    @Override
    public MassStep ellipsoid(double x, double y, double z) {
      inertiaTensor = new EllipsoidInertiaTensor(x, y, z);
      return this;
    }

    @Override
    public MassStep solid(boolean solid) {
      this.solid = solid;
      return this;
    }

    @Override
    public PositionStep mass(double mass) {
      if (mass == 0.0) {
        throw new IllegalArgumentException("Zero mass is not allowed");
      }
      return inverseMass(1 / mass);
    }

    @Override
    public PositionStep inverseMass(double inverseMass) {
      this.inverseMass = inverseMass;
      return this;
    }

    @Override
    public OrientationStep position(Vector3D position) {
      this.position = new Vector3D(position);
      return this;
    }

    @Override
    public OrientationStep position(double x, double y, double z) {
      this.position = new Vector3D(x, y, z);
      return this;
    }

    @Override
    public VelocityStep orientation(Quaternion orientation) {
      this.orientation = new Quaternion(orientation);
      return this;
    }

    @Override
    public VelocityStep orientation(double r, double i, double j, double k) {
      this.orientation = new Quaternion(r, i, j, k);
      return this;
    }

    @Override
    public AccelerationStep velocity(Vector3D velocity) {
      this.velocity = new Vector3D(velocity);
      return this;
    }

    @Override
    public AccelerationStep velocity(double x, double y, double z) {
      this.velocity = new Vector3D(x, y, z);
      return this;
    }

    @Override
    public FinalStep acceleration(Vector3D acceleration) {
      this.acceleration = new Vector3D(acceleration);
      return this;
    }

    @Override
    public FinalStep acceleration(double x, double y, double z) {
      this.acceleration = new Vector3D(x, y, z);
      return this;
    }

    @Override
    public RigidBody build() {
      RigidBody body = new RigidBody(position, orientation);
      body.setInverseMass(inverseMass);
      body.setVelocity(velocity.getX(), velocity.getY(), velocity.getZ());
      body.setAcceleration(acceleration.getX(), acceleration.getY(), acceleration.getZ());
      return setInertiaTensor(body);
    }

    private RigidBody setInertiaTensor(RigidBody body) {
      if (inertiaTensor instanceof CuboidInertiaTensor) {
        CuboidInertiaTensor tensor = (CuboidInertiaTensor) inertiaTensor;
        body.setInertiaTensorCuboid(tensor.x, tensor.y, tensor.z);
      } else if (inertiaTensor instanceof SphereInertiaTensor) {
        SphereInertiaTensor tensor = (SphereInertiaTensor) inertiaTensor;
        if (solid) {
          body.setInertiaTensorSphereSolid(tensor.radius);
        } else {
          body.setInertiaTensorSphereHollow(tensor.radius);
        }
      } else if (inertiaTensor instanceof CylinderInertiaTensor) {
        CylinderInertiaTensor tensor = (CylinderInertiaTensor) inertiaTensor;
        if (solid) {
          body.setInertiaTensorCylinderSolid(tensor.height, tensor.radius);
        } else {
          body.setInertiaTensorCylinderHollow(tensor.height, tensor.radius, tensor.radius * 0.9);
        }
      } else if (inertiaTensor instanceof HemisphereInertiaTensor) {
        HemisphereInertiaTensor tensor = (HemisphereInertiaTensor) inertiaTensor;
        body.setInertiaTensorHemisphere(tensor.radius);
      } else if (inertiaTensor instanceof EllipsoidInertiaTensor) {
        EllipsoidInertiaTensor tensor = (EllipsoidInertiaTensor) inertiaTensor;
        body.setInertiaTensorEllipsoid(tensor.x, tensor.y, tensor.z);
      } else {
        throw new IllegalStateException(
            "Unknown tensor type: "
                + (inertiaTensor == null ? null : inertiaTensor.getClass().getSimpleName()));
      }
      return body;
    }
  }

  /**
   * Step for defining the shape of the body. This is important for defining its inertia tensor.
   *
   * @author Benedikt Full
   */
  public interface ShapeStep {

    /**
     * Defines the shape of the body as a cuboid of the given dimensions.
     *
     * @param x edge length along the x axis in meters
     * @param y edge length along the y axis in meters
     * @param z edge length along the z axis in meters
     * @return the step for defining the cuboid's mass
     * @see #cube(double)
     * @see #defaultRestingCube()
     */
    MassStep cuboid(double x, double y, double z);

    /**
     * Defines the shape of the body as a cube with the given edge length.
     *
     * @param edgeLength the length of the cube's edges in meters
     * @return the step for defining the cube's mass
     * @see #cuboid(double, double, double)
     * @see #defaultRestingCube()
     */
    default MassStep cube(double edgeLength) {
      return cuboid(edgeLength, edgeLength, edgeLength);
    }

    /**
     * Configures a cube with an edge length of 1 meter, a mass of 1 kilogram, located at the
     * origin, default orientation, and resting.
     *
     * @return the step for building the body object
     * @see #cuboid(double, double, double)
     * @see #cube(double)
     */
    default FinalStep defaultRestingCube() {
      return cube(1).mass(1).locatedAtOrigin().defaultOrientation().resting();
    }

    /**
     * Defines the shape of the body as a sphere with the given radius.
     *
     * @param radius the sphere radius in meters
     * @return the step for defining whether the sphere is hollow or solid
     */
    FillStep sphere(double radius);

    /**
     * Defines the shape of the body as a cylinder with the given properties.
     *
     * @param height the height in meters
     * @param radius the radius in meters
     * @return the step for defining whether the cylinder is hollow or solid
     */
    FillStep cylinder(double height, double radius);

    /**
     * Defines the shape of the body as a hemisphere with the given radius.
     *
     * @param radius the radius in meters
     * @return the step for defining the hemisphere's mass
     */
    MassStep hemisphere(double radius);

    /**
     * Defines the shape of the body as an ellipsoid with the given dimensions.
     *
     * @param x the radius along the x axis in meters
     * @param y the radius along the y axis in meters
     * @param z the radius along the z axis in meters
     * @return the step for defining the ellipsoid's mass
     */
    MassStep ellipsoid(double x, double y, double z);
  }

  /**
   * Optional step for defining whether a body is hollow or solid.
   *
   * @author Benedikt Full
   */
  public interface FillStep {

    /**
     * Defines whether the body is solid or hollow
     *
     * @param solid {@code true} if the body is meant to be solid, {@code false} otherwise
     * @return the step for defining the body's mass
     */
    MassStep solid(boolean solid);

    /**
     * Defines the body as a solid object, meaning its mass is evenly distributed.
     *
     * @return the step for defining the body's mass
     * @see #hollow()
     */
    default MassStep solid() {
      return solid(true);
    }

    /**
     * Defines that the body is hollow.
     *
     * <p><b>Note:</b> This method takes no parameters and therefore implementations must make
     * assumptions about what parts of the body are actually hollow.
     *
     * @return the step for defining the body's mass
     * @see #solid()
     */
    default MassStep hollow() {
      return solid(false);
    }
  }

  /**
   * Step for defining the mass of the body.
   *
   * @author Benedikt Full
   */
  public interface MassStep {

    /**
     * Sets the mass of the body to the given value.
     *
     * @param mass a non-zero value representing the mass of the body in kilograms
     * @return the step for defining the body's position in the world
     */
    PositionStep mass(double mass);

    /**
     * Sets the inverse mass of the body to the given value.
     *
     * @param inverseMass the inverse mass value
     * @return the step for defining the body's position in the world
     */
    PositionStep inverseMass(double inverseMass);

    /**
     * Sets the mass of the body to an infinite value. This is useful for bodies that are meant to
     * remain stationary.
     *
     * @return the step for defining the body's position in the world
     */
    default PositionStep infiniteMass() {
      return inverseMass(0.0);
    }
  }

  /**
   * Step for defining the position of the body in 3D space.
   *
   * @author Benedikt Full
   */
  public interface PositionStep {

    /**
     * Sets the position of the body to the given coordinates.
     *
     * @param position the position of the body
     * @return the step for defining the body's orientation
     */
    OrientationStep position(Vector3D position);

    /**
     * Sets the position of the body to the given coordinates.
     *
     * @param x the x coordinate
     * @param y the y coordinate
     * @param z the z coordinate
     * @return the step for defining the body's orientation
     */
    OrientationStep position(double x, double y, double z);

    /**
     * Positions the body at the origin of the coordinate system.
     *
     * @return the step for defining the body's orientation
     */
    default OrientationStep locatedAtOrigin() {
      return position(new Vector3D());
    }
  }

  /**
   * Step for defining the orientation of the body in 3D space.
   *
   * @author Benedikt Full
   */
  public interface OrientationStep {

    /**
     * Sets the orientation of the body to the given value.
     *
     * @param orientation the orientation of the body
     * @return the step for defining the velocity of the body
     */
    VelocityStep orientation(Quaternion orientation);

    /**
     * Sets the orientation of the body to the given value.
     *
     * @param r the real component of the orientation
     * @param i the first complex component
     * @param j the second complex component
     * @param k the third complex component
     * @return the step for defining the velocity of the body
     */
    VelocityStep orientation(double r, double i, double j, double k);

    /**
     * Sets the orientation of the body to a default value.
     *
     * @return the step for defining the velocity of the body
     */
    default VelocityStep defaultOrientation() {
      return orientation(new Quaternion());
    }
  }

  /**
   * Step for defining the velocity of the body.
   *
   * @author Benedikt Full
   */
  public interface VelocityStep {

    /**
     * Sets the velocity of the body to the given value.
     *
     * @param velocity the velocity of the body
     * @return the step for defining the acceleration experienced by the body
     */
    AccelerationStep velocity(Vector3D velocity);

    /**
     * Sets the velocity of the body to the given value.
     *
     * @param x the x component
     * @param y the y component
     * @param z the z component
     * @return the step for defining the acceleration experienced by the body
     */
    AccelerationStep velocity(double x, double y, double z);

    /**
     * Sets the velocity of the body to zero.
     *
     * @return the step for defining the acceleration experienced by the body
     */
    default AccelerationStep noVelocity() {
      return velocity(new Vector3D());
    }

    /**
     * Sets both the velocity and acceleration of the body to zero, therefore making it a
     * <i>resting</i> body.
     *
     * @return the step for building the body object
     */
    default FinalStep resting() {
      return noVelocity().noAcceleration();
    }
  }

  /**
   * Step for defining the acceleration experienced by the body.
   *
   * @author Benedikt Full
   */
  public interface AccelerationStep {

    /**
     * Sets the acceleration experienced by the body to the given value.
     *
     * @param acceleration the acceleration of the body
     * @return the step for building the body object
     */
    FinalStep acceleration(Vector3D acceleration);

    /**
     * Sets the acceleration experienced by the body to the given value.
     *
     * @param x the x component of the acceleration
     * @param y the y component of the acceleration
     * @param z the z component of the acceleration
     * @return the step for building the body object
     */
    FinalStep acceleration(double x, double y, double z);

    /**
     * Sets the acceleration experienced by the body to zero.
     *
     * @return the step for building the body object
     */
    default FinalStep noAcceleration() {
      return acceleration(new Vector3D());
    }
  }

  /**
   * Step for creating the actual rigid body based on the state of the builder.
   *
   * @author Benedikt Full
   */
  public interface FinalStep {

    /** @return a new rigid body based on the state of the builder */
    RigidBody build();
  }

  /**
   * Marker interface for classes defining inertia tensors for specific types of physical objects.
   *
   * @author Benedikt Full
   */
  private interface InertiaTensor {}

  private static class CuboidInertiaTensor implements InertiaTensor {
    private final double x;
    private final double y;
    private final double z;

    private CuboidInertiaTensor(double x, double y, double z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
  }

  private static class SphereInertiaTensor implements InertiaTensor {
    private final double radius;

    private SphereInertiaTensor(double radius) {
      this.radius = radius;
    }
  }

  private static class CylinderInertiaTensor implements InertiaTensor {

    private final double height;
    private final double radius;

    private CylinderInertiaTensor(double height, double radius) {
      this.height = height;
      this.radius = radius;
    }
  }

  private static class HemisphereInertiaTensor implements InertiaTensor {
    private final double radius;

    private HemisphereInertiaTensor(double radius) {
      this.radius = radius;
    }
  }

  private static class EllipsoidInertiaTensor implements InertiaTensor {
    private final double x;
    private final double y;
    private final double z;

    private EllipsoidInertiaTensor(double x, double y, double z) {
      this.x = x;
      this.y = y;
      this.z = z;
    }
  }
}
